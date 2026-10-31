#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>
#include "libRayfem/frame/render.hpp"

namespace RayFEM {
class Server {
public:
    // control_port=0 keeps a plain streaming receiver. The desktop viewer uses
    // a separate identity/readiness/lifecycle endpoint for init/shutdown.
    Server(const std::string& listen, size_t port, size_t control_port = 0);
    ~Server();
    Server(const Server&) = delete;
    Server& operator=(const Server&) = delete;

    // Call on the rendering thread. Network reception never calls MFEM.
    // The budget bounds decode work per UI tick; complete frames remain atomic.
    // While 64 frames (or 128 MiB) wait to be fetched, the server stops
    // reading the stream, so senders wait instead of frames being dropped.
    // fetchLatestFrame drops all but the newest, counted as dropped.
    std::vector<std::shared_ptr<RenderFrame>> fetchFrames(size_t max_frames = 4);
    std::shared_ptr<RenderFrame> fetchLatestFrame();
    size_t getReceivedFrameCount() const;
    size_t getDroppedFrameCount() const;
    size_t getErrorCount() const;
    size_t getPendingFrameCount() const;
    // While not receiving, frames are counted and dropped (including any
    // already queued); mesh registrations are still kept, so frames that
    // arrive after receiving resumes decode normally.
    void setReceiving(bool receiving);
    bool receiving() const;
    size_t getSkippedFrameCount() const;
    bool ready() const;
    // Only mark ready after the desktop graphics/UI initialization succeeds.
    void markViewerReady(bool ready = true);
    // A verified lifecycle request asks the host's main loop to exit normally.
    // The host must observe this flag and destroy the Server after UI cleanup.
    bool shutdownRequested() const;
    std::string getLastError() const;
private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};
} // namespace RayFEM
