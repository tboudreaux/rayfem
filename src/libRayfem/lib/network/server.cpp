#include "libRayfem/frame/render.hpp"
#include "libRayfem/network/server.hpp"
#include "libRayfem/network/zmq.hpp"

#include <optional>
#include <thread>
#include <mutex>
#include <deque>

#include "czmq.h"

namespace RayFEM {
    Server::Server(
        const std::string& listen,
        const size_t port
    ) :
    m_listen(listen),
    m_port(port),
    m_running(true),
    m_worker(std::thread(&Server::networkLoop, this)) {}

    Server::~Server() {
        m_running = false;
        if (m_worker.joinable()) {
            m_worker.join();
        }
    }

    std::unique_ptr<RenderFrame> Server::fetchLatestFrame() {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_frameQueue.empty()) return nullptr;

        auto frame = std::move(m_frameQueue.back());
        m_frameQueue.clear();
        return frame;
    }

    void Server::networkLoop() {
        std::string address = "tcp://" + m_listen + ":" + std::to_string(m_port);

        SocketPtr pull(zsock_new_pull(address.c_str()));

        zpoller_t *poller = zpoller_new(pull.get(), NULL);

        while (m_running) {
            void *which = zpoller_wait(poller, 100);
            if (!which) continue;

            MsgPtr msg(zmsg_recv(pull.get()));
            if (!msg) continue;

            auto frame = std::make_unique<RenderFrame>();

            zframe_t *h_frame = zmsg_pop(msg.get());
            if (zframe_size(h_frame) == sizeof(PacketHeader)) {
                frame->metadata = reinterpret_cast<PacketHeader *>(zframe_data(h_frame))->metadata;
            }

            zframe_destroy(&h_frame);

            zframe_t *m_frame = zmsg_pop(msg.get());
            zframe_t *g_frame = zmsg_pop(msg.get());

            if (m_frame && g_frame) {
                std::string m_data(reinterpret_cast<char *>(zframe_data(m_frame)), zframe_size(m_frame));
                std::string g_data(reinterpret_cast<char *>(zframe_data(g_frame)), zframe_size(g_frame));

                std::cout << g_data << std::endl;

                std::stringstream m_ss(m_data);
                std::stringstream g_ss(g_data);

                try {
                    frame->mesh = std::make_unique<mfem::Mesh>(m_ss);
                    frame->mesh->EnsureNodes();
                    frame->gf = std::make_unique<mfem::GridFunction>(frame->mesh.get(), g_ss);
                } catch (...) {
                    std::cerr << "Error creating mesh frame" << std::endl;
                }

                ++m_frameCount;

                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    m_frameQueue.push_back(std::move(frame));
                }
            }

            zframe_destroy(&m_frame);
            zframe_destroy(&g_frame);

        }
        zpoller_destroy(&poller);
    }
}
