#pragma once

#include <memory>
#include <cstdint>
#include <atomic>
#include <thread>
#include <mutex>
#include <deque>
#include <optional>

#include "libRayfem/frame/render.hpp"

namespace RayFEM {
    class Server {
    public:
        Server(const std::string& listen, size_t port);
        ~Server();

        std::unique_ptr<RenderFrame> fetchLatestFrame();
        size_t getReceivedFrameCount() const { return m_frameCount.load(); }
    private:
        std::string m_listen;
        int m_port;
        std::atomic<bool> m_running;
        std::atomic<size_t> m_frameCount{0};
        std::thread m_worker;
        std::mutex m_mutex;
        std::deque<std::unique_ptr<RenderFrame>> m_frameQueue;
    private:
        void networkLoop();
    };
}