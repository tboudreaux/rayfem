#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <thread>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include "libRayfem/network/packet.hpp"

namespace RayFEM {
// One field as it travels: the text MFEM's FiniteElementSpace::Save writes,
// and the coefficients in that space's order. Nothing is copied before
// encoding; values must stay valid during the send call.
struct FieldData {
    std::string_view name;
    std::string_view space;
    std::span<const double> values;
};

// Sending in the background (see StreamClient::startQueue). max_bytes bounds the encoded
// packets queued and not yet written to the network; a packet larger
// than that is queued alone. When a frame does not fit, Wait blocks the
// send until it does, DropOldest discards the oldest queued frames, and
// DropNewest refuses the frame. Meshes and schemas always wait.
struct StreamQueue {
    enum class WhenFull { Wait, DropOldest, DropNewest };
    size_t max_bytes = size_t{512} << 20;
    WhenFull when_full = WhenFull::Wait;
};
struct StreamQueueStats {
    size_t pending = 0;        // packets waiting for the background thread
    size_t pending_bytes = 0;  // bytes encoded and not yet written to the network
    size_t sent = 0;           // packets handed to the network
    size_t dropped = 0;        // frames discarded or refused because the queue was full
    size_t failed = 0;         // packets that could not be sent
};


// Streams meshes and frames without MFEM: meshes are MFEM mesh text and
// fields are FieldData. This is what language bindings and other producers
// use; Client (client.hpp) serializes MFEM objects and sends through it.
// The viewer validates everything it decodes; this checks what it can
// without MFEM. One StreamClient must be used from one thread.
//
// Streaming is one-way, but the client watches its connection: a send is
// refused when no viewer is connected (after waiting up to a second for one),
// and a frame is refused when the viewer's listener has restarted since its
// mesh was sent (as when the viewer changes its host or port and comes back
// on this one), since that listener no longer has the mesh. A packet sent in
// the moment the viewer moves can still be lost; the next send reports it.
//
// Each send encodes its packet once, copying the values, so the caller may
// change them as soon as the call returns. Without a queue (the default) the
// call then hands the packet to the socket, waiting for a viewer as above.
// With a queue (startQueue) the call returns once the packet is queued, and
// a background thread sends packets in order, so a solver need not wait for
// the viewer or the network. A packet the background thread cannot send
// (no viewer, or a frame whose mesh the viewer no longer has) is counted in
// queueStats, and the next send reports it by being refused, as without a
// queue; packets queued after it are still sent.
class StreamClient {
public:
    StreamClient(const std::string& host, size_t port);
    ~StreamClient();
    StreamClient(StreamClient&&) noexcept;
    StreamClient& operator=(StreamClient&&) noexcept;
    StreamClient(const StreamClient&) = delete;
    StreamClient& operator=(const StreamClient&) = delete;

    // See Client::registerSchema.
    bool registerSchema(const FrameSchema& schema);
    const std::optional<FrameSchema>& schema() const { return m_schema; }
    // mesh_text is MFEM mesh text ("MFEM mesh v1.0", NURBS or NC).
    bool sendMesh(uint64_t mesh_id, std::string_view mesh_text);
    bool sendFrame(uint64_t mesh_id, std::span<const FieldData> fields, const FrameMetadata& metadata,
                   const FieldData* displacement = nullptr);
    // Binary fields (the default) or MFEM GridFunction text, for viewers
    // older than the binary encoding.
    void setBinaryFields(bool binary) { m_binaryFields = binary; }
    bool binaryFields() const { return m_binaryFields; }
    const std::string& getLastError() const { return m_lastError; }
    // Whether a viewer is connected now; "host:port" it streams to.
    bool connected();
    const std::string& address() const;
    bool fail(std::string message);

    // Sending in the background (see above and StreamQueue).
    using Queue = StreamQueue;
    using QueueStats = StreamQueueStats;
    // Starts sending in the background, or changes the options of a queue.
    void startQueue(const Queue& options = {});
    // Sends what is queued, waiting up to timeout_ms (dropping what is left
    // after that), then returns to sending in the caller's thread. The
    // destructor does this with a five-second limit.
    void stopQueue(int timeout_ms = 5000);
    bool queued() const { return m_worker.joinable(); }
    // Waits until every packet is written to the network (or failed);
    // negative waits indefinitely. False on timeout.
    bool flush(int timeout_ms = -1);
    QueueStats queueStats() const;

private:
    struct Registration { uint64_t revision = 0; uint32_t last_frame = 0; bool has_frame = false; };
    struct State;
    struct Packet;
    Packet makePacket(Protocol::Header header, size_t payload_bytes);
    bool submit(Packet packet);
    bool reachable(Protocol::Kind kind, uint64_t mesh_id);
    bool backgroundFailure();
    bool sendSchema();
    struct Socket;
    std::unique_ptr<Socket> m_socket;
    std::shared_ptr<State> m_state;
    std::thread m_worker;
    uint64_t m_sessionId = 0, m_nextRevision = 0, m_schemaRevision = 0;
    std::unordered_map<uint64_t, Registration> m_meshes;
    std::optional<FrameSchema> m_schema;
    std::string m_lastError;
    bool m_binaryFields = true;
};
} // namespace RayFEM
