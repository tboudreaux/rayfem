#include "libRayfem/network/stream.hpp"
#include "libRayfem/network/zmq.hpp"

#include <algorithm>
#include <atomic>
#include <bit>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <cmath>
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <random>
#include <stdexcept>
#include <unordered_set>

namespace RayFEM {
namespace {
// How long a send without a queue waits for a connected viewer that is busy.
constexpr int BusyMilliseconds = 30000;
bool validSpaceHeader(std::string_view space) {
    return (space.starts_with("FiniteElementSpace\n") || space.starts_with("MFEM FiniteElementSpace v1.0\n")) &&
           space.size() <= Protocol::MaxSpaceHeaderBytes && space.find('\0') == std::string_view::npos;
}
bool finite(std::span<const double> values) {
    for (double v : values) if (!std::isfinite(v)) return false;
    return true;
}
// Fills a packet's buffer: integers big-endian as in packet.hpp, and each
// double as the big-endian bits of its value.
struct Writer {
    char* at;
    void integer(uint64_t value, size_t bytes) {
        for (size_t i = bytes; i > 0; --i) *at++ = static_cast<char>(value >> ((i - 1) * 8));
    }
    void bytes(std::string_view data) { std::memcpy(at, data.data(), data.size()); at += data.size(); }
    // One pass that also checks the values; false when one is not finite.
    bool doubles(std::span<const double> values) {
        constexpr uint64_t exponent = 0x7ff0000000000000ull;
        uint64_t nonfinite = 0;
        for (size_t i = 0; i < values.size(); ++i) {
            uint64_t bits = std::bit_cast<uint64_t>(values[i]);
            nonfinite |= static_cast<uint64_t>((bits & exponent) == exponent);
            if constexpr (std::endian::native == std::endian::little) bits = std::byteswap(bits);
            std::memcpy(at + 8 * i, &bits, sizeof(bits));
        }
        at += 8 * values.size();
        return !nonfinite;
    }
};
// "RFB1", the header length and text, the value count, then IEEE doubles.
size_t binarySize(const FieldData& field) { return 4 + 4 + field.space.size() + 8 + 8 * field.values.size(); }
bool writeBinary(Writer& out, const FieldData& field) {
    out.bytes(Protocol::BinaryFieldMagic);
    out.integer(field.space.size(), 4);
    out.bytes(field.space);
    out.integer(field.values.size(), 8);
    return out.doubles(field.values);
}
// MFEM GridFunction::Save text: the space header, then one value per line.
std::string encodeText(const FieldData& field) {
    std::string out(field.space);
    if (!out.empty() && out.back() != '\n') out += '\n';
    out += '\n';
    char number[32];
    for (double value : field.values) {
        const int n = std::snprintf(number, sizeof(number), "%.17g\n", value);
        out.append(number, static_cast<size_t>(n));
    }
    return out;
}
}

// An encoded packet: the header and payload in one buffer.
struct StreamClient::Packet {
    struct Buffer {
        std::unique_ptr<char[]> data;
        size_t size = 0;
        std::shared_ptr<State> state;
        Buffer(size_t bytes, std::shared_ptr<State> owner);
        // Stops counting it, with the state's lock held.
        void forget();
        ~Buffer();
    };
    Protocol::Kind kind = Protocol::Kind::Frame;
    uint64_t mesh_id = 0;
    uint32_t frame_index = 0;
    uint64_t sequence = 0;
    std::unique_ptr<Buffer> buffer;
};

// What the caller and the background thread share: the queue, and the bytes
// of every encoded packet not yet written to the network. A packet's buffer
// counts until it is destroyed, which libzmq does once it has written it.
struct StreamClient::State {
    std::mutex mutex;
    std::condition_variable changed;
    Queue options;
    size_t bytes = 0;
    std::deque<Packet> queue;
    QueueStats counts;
    bool stopping = false;
    // The latest background failure, and the packet it came from; the caller
    // reports a failure once, and not again for packets queued before then.
    std::string error;
    uint64_t submitted = 0, error_sequence = 0, reported_through = 0;
};
StreamClient::Packet::Buffer::Buffer(size_t bytes, std::shared_ptr<State> owner)
    : data(std::make_unique_for_overwrite<char[]>(bytes)), size(bytes), state(std::move(owner)) {
    std::lock_guard lock(state->mutex);
    state->bytes += size;
}
void StreamClient::Packet::Buffer::forget() { state->bytes -= size; state.reset(); }
StreamClient::Packet::Buffer::~Buffer() {
    if (!state) return;
    { std::lock_guard lock(state->mutex); state->bytes -= size; }
    state->changed.notify_all();
}

// The PUSH socket, and libzmq's monitor of its connection to the viewer.
// Streaming is one-way, so the connection is the only feedback: a send with
// no connected viewer is refused instead of queued, and a reconnection (the
// viewer's listener closed and opened again, for example after changing its
// host on the same port) starts a new epoch whose listener has none of the
// meshes sent before it. Used from one thread at a time: the caller's, or the
// background thread's while a queue runs.
struct StreamClient::Socket {
    // The first member: the host's signal handlers are kept before CZMQ starts.
    bool keep_signals = (KeepHostSignals(), true);
    SocketPtr value{zsock_new(ZMQ_PUSH)};
    SocketPtr monitor;
    std::string address;
    std::atomic<bool> connected = false;
    uint64_t epoch = 0;  // connections made so far
    std::unordered_map<uint64_t, uint64_t> mesh_epochs;  // the epoch each mesh was sent in

    ~Socket() { if (value && monitor) zmq_socket_monitor(zsock_resolve(value), nullptr, 0); }
    // Apply connection events; wait up to wait_ms for a connection when there is none.
    void poll(int wait_ms) {
        if (!monitor) return;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_ms);
        for (;;) {
            long timeout = 0;
            if (!connected) timeout = std::max<long>(0, std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now()).count());
            zmq_pollitem_t item{zsock_resolve(monitor), 0, ZMQ_POLLIN, 0};
            if (zmq_poll(&item, 1, timeout) <= 0) return;
            zmsg_t* message = zmsg_recv(monitor);
            if (!message) return;
            zframe_t* event = zmsg_first(message);
            if (event && zframe_size(event) >= 2) {
                uint16_t id;
                std::memcpy(&id, zframe_data(event), sizeof(id));
                if (id == ZMQ_EVENT_CONNECTED) { connected = true; ++epoch; }
                else if (id == ZMQ_EVENT_DISCONNECTED) connected = false;
            }
            zmsg_destroy(&message);
        }
    }
    // Whether a packet of this kind can go now, or why not; wait_ms is how
    // long to wait for a viewer when none is connected.
    bool ready(Protocol::Kind kind, uint64_t mesh_id, int wait_ms, std::string& error) {
        if (!monitor) return true;
        poll(connected ? 0 : wait_ms);
        if (!connected) {
            error = "No viewer is listening on " + address +
                    "; it may have moved to another host or port, or closed. Nothing was sent.";
            return false;
        }
        if (kind != Protocol::Kind::Frame) return true;
        const auto found = mesh_epochs.find(mesh_id);
        if (found == mesh_epochs.end()) {
            error = "Mesh " + std::to_string(mesh_id) + " did not reach the viewer on " + address +
                    ". Send the mesh again; nothing was sent.";
            return false;
        }
        if (found->second != epoch) {
            error = "The viewer on " + address + " stopped listening and started again since mesh " +
                    std::to_string(mesh_id) + " was sent (it may have changed its host or port), so it "
                    "no longer has the mesh. Send the mesh again; nothing was sent.";
            return false;
        }
        return true;
    }
    // Hands the packet to libzmq without copying it, or says why not. A
    // viewer whose queue is full stops reading, so the send waits while the
    // viewer stays connected: up to busy_ms (negative: without limit) and
    // until stopped() says to give up.
    template <class Stopped>
    bool send(Packet& packet, int wait_ms, int busy_ms, Stopped stopped, std::string& error) {
        if (!ready(packet.kind, packet.mesh_id, wait_ms, error)) return false;
        Packet::Buffer* buffer = packet.buffer.release();
        zmq_msg_t message;
        if (zmq_msg_init_data(&message, buffer->data.get(), buffer->size,
                              [](void*, void* hint) { delete static_cast<Packet::Buffer*>(hint); }, buffer) != 0) {
            delete buffer;
            error = "Out of memory sending RayFEM packet";
            return false;
        }
        const auto start = std::chrono::steady_clock::now();
        while (zmq_msg_send(&message, zsock_resolve(value), 0) < 0) {
            const int why = zmq_errno();
            if (why == EAGAIN && monitor) poll(0);
            if (why != EAGAIN) error = std::string("Cannot send RayFEM packet: ") + zmq_strerror(why);
            else if (monitor && !connected)
                error = "The viewer on " + address + " went away before taking the packet. Nothing was sent.";
            else if (stopped()) error = "The send was stopped before the viewer took the packet.";
            else if (busy_ms >= 0 && std::chrono::steady_clock::now() - start > std::chrono::milliseconds(busy_ms))
                error = "The viewer on " + address + " took no packet for " + std::to_string(busy_ms / 1000) +
                        " s; it is busy or stalled. Nothing was sent.";
            else continue;
            zmq_msg_close(&message);
            return false;
        }
        if (packet.kind == Protocol::Kind::Mesh) mesh_epochs[packet.mesh_id] = epoch;
        return true;
    }
};
StreamClient::~StreamClient() { if (m_worker.joinable()) stopQueue(); }
StreamClient::StreamClient(StreamClient&& other) noexcept = default;
StreamClient& StreamClient::operator=(StreamClient&& other) noexcept {
    if (this == &other) return *this;
    if (m_worker.joinable()) stopQueue();
    m_socket = std::move(other.m_socket);
    m_state = std::move(other.m_state);
    m_worker = std::move(other.m_worker);
    m_sessionId = other.m_sessionId;
    m_nextRevision = other.m_nextRevision;
    m_schemaRevision = other.m_schemaRevision;
    m_meshes = std::move(other.m_meshes);
    m_schema = std::move(other.m_schema);
    m_lastError = std::move(other.m_lastError);
    m_binaryFields = other.m_binaryFields;
    return *this;
}
StreamClient::StreamClient(const std::string& host, size_t port)
    : m_socket(std::make_unique<Socket>()), m_state(std::make_shared<State>()) {
    if (!m_socket->value || port == 0 || port > 65535) throw std::runtime_error("Invalid RayFEM client endpoint");
    zsock_set_linger(m_socket->value, 1000);
    // Short attempts, so a send waiting on a busy viewer notices it leave.
    zsock_set_sndtimeo(m_socket->value, 250);
    // A few packets wait in libzmq; beyond that a send waits (or the queue holds them).
    zsock_set_sndhwm(m_socket->value, 8);
    // Queue only on a completed connection, so nothing waits for a viewer that has gone.
    zsock_set_immediate(m_socket->value, 1);
    std::random_device random;
    m_sessionId = (static_cast<uint64_t>(random()) << 32) ^ random()
        ^ static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
    if (!m_sessionId) m_sessionId = 1;
    // The monitor starts before connecting, so it sees the first connection.
    const std::string events = "inproc://rayfem-client-" + std::to_string(m_sessionId) + "-" +
        std::to_string(reinterpret_cast<uintptr_t>(m_socket.get()));
    if (zmq_socket_monitor(zsock_resolve(m_socket->value), events.c_str(), ZMQ_EVENT_CONNECTED | ZMQ_EVENT_DISCONNECTED) == 0) {
        m_socket->monitor = SocketPtr(zsock_new(ZMQ_PAIR));
        if (m_socket->monitor && zsock_connect(m_socket->monitor, "%s", events.c_str()) != 0) m_socket->monitor = SocketPtr();
    }
    m_socket->address = host + ":" + std::to_string(port);
    const std::string address = "tcp://" + m_socket->address;
    if (zsock_connect(m_socket->value, "%s", address.c_str()) != 0)
        throw std::runtime_error("Cannot connect RayFEM client to " + address);
}
bool StreamClient::connected() {
    if (!m_socket->monitor) return true;
    // While a queue runs, its thread watches the connection.
    if (!m_worker.joinable()) m_socket->poll(0);
    return m_socket->connected;
}
const std::string& StreamClient::address() const { return m_socket->address; }
bool StreamClient::fail(std::string message) { m_lastError = std::move(message); return false; }

void StreamClient::startQueue(const Queue& options) {
    {
        std::lock_guard lock(m_state->mutex);
        m_state->options = options;
    }
    if (m_worker.joinable()) return;
    m_state->stopping = false;
    // The thread sends packets in order. After waiting in vain for a viewer it
    // stops waiting, so a closed viewer costs queued packets no time, until
    // a viewer connects again.
    m_worker = std::thread([socket = m_socket.get(), state = m_state] {
        bool waited = false;
        for (;;) {
            Packet packet;
            {
                std::unique_lock lock(state->mutex);
                state->changed.wait(lock, [&] { return !state->queue.empty() || state->stopping; });
                if (state->queue.empty()) return;
                packet = std::move(state->queue.front());
                state->queue.pop_front();
            }
            std::string error;
            const bool sent = socket->send(packet, waited ? 0 : 1000, -1, [&] {
                std::lock_guard lock(state->mutex);
                return state->stopping;
            }, error);
            waited = !sent && !socket->connected;
            packet.buffer.reset();
            {
                std::lock_guard lock(state->mutex);
                if (sent) ++state->counts.sent;
                else {
                    ++state->counts.failed;
                    const char* kind = packet.kind == Protocol::Kind::Frame ? "frame" :
                                       packet.kind == Protocol::Kind::Mesh ? "mesh" : "schema";
                    state->error = std::string("A queued ") + kind +
                        (packet.kind == Protocol::Kind::Frame ? " (index " + std::to_string(packet.frame_index) + ", mesh " +
                         std::to_string(packet.mesh_id) + ")" : packet.kind == Protocol::Kind::Mesh ? " (" +
                         std::to_string(packet.mesh_id) + ")" : "") + " was not sent: " + error;
                    state->error_sequence = packet.sequence;
                }
            }
            state->changed.notify_all();
        }
    });
}
void StreamClient::stopQueue(int timeout_ms) {
    if (!m_worker.joinable()) return;
    flush(timeout_ms);
    std::deque<Packet> left;
    {
        std::lock_guard lock(m_state->mutex);
        left.swap(m_state->queue);
        for (auto& packet : left) packet.buffer->forget();
        m_state->counts.dropped += left.size();
        m_state->stopping = true;
    }
    m_state->changed.notify_all();
    m_worker.join();
    m_state->stopping = false;
}
bool StreamClient::flush(int timeout_ms) {
    std::unique_lock lock(m_state->mutex);
    const auto empty = [&] { return m_state->bytes == 0; };
    if (timeout_ms < 0) { m_state->changed.wait(lock, empty); return true; }
    return m_state->changed.wait_for(lock, std::chrono::milliseconds(timeout_ms), empty);
}
StreamClient::QueueStats StreamClient::queueStats() const {
    std::lock_guard lock(m_state->mutex);
    QueueStats stats = m_state->counts;
    stats.pending = m_state->queue.size();
    stats.pending_bytes = m_state->bytes;
    return stats;
}
bool StreamClient::reachable(Protocol::Kind kind, uint64_t mesh_id) {
    if (m_worker.joinable()) return true;
    std::string error;
    return m_socket->ready(kind, mesh_id, 1000, error) || fail(std::move(error));
}
bool StreamClient::backgroundFailure() {
    if (!m_worker.joinable()) return false;
    std::lock_guard lock(m_state->mutex);
    if (m_state->error_sequence <= m_state->reported_through) return false;
    m_state->reported_through = m_state->submitted;
    m_lastError = m_state->error;
    return true;
}

StreamClient::Packet StreamClient::makePacket(Protocol::Header header, size_t payload_bytes) {
    header.payload_bytes = payload_bytes;
    header.session_id = m_sessionId;
    Packet packet;
    packet.kind = header.kind;
    packet.mesh_id = header.mesh_id;
    packet.frame_index = header.metadata.frame_index;
    packet.buffer = std::make_unique<Packet::Buffer>(Protocol::HeaderBytes + payload_bytes, m_state);
    const std::string encoded = Protocol::encodeHeader(header);
    std::memcpy(packet.buffer->data.get(), encoded.data(), Protocol::HeaderBytes);
    return packet;
}
bool StreamClient::submit(Packet packet) {
    if (!m_worker.joinable()) {
        std::string error;
        if (!m_socket->send(packet, 1000, BusyMilliseconds, [] { return false; }, error)) return fail(std::move(error));
        m_lastError.clear();
        return true;
    }
    std::unique_lock lock(m_state->mutex);
    const Queue& options = m_state->options;
    const size_t size = packet.buffer->size;
    // The state counts this packet already; one larger than the limit goes alone.
    const auto fits = [&] { return m_state->bytes <= options.max_bytes || m_state->bytes == size; };
    const bool frame = packet.kind == Protocol::Kind::Frame;
    if (frame && options.when_full == Queue::WhenFull::DropNewest && !fits()) {
        packet.buffer->forget();
        ++m_state->counts.dropped;
        return fail("The send queue is full (" + std::to_string(options.max_bytes >> 20) +
                    " MiB waiting for the viewer); the frame was dropped.");
    }
    while (!fits()) {
        if (frame && options.when_full == Queue::WhenFull::DropOldest) {
            const auto oldest = std::find_if(m_state->queue.begin(), m_state->queue.end(),
                                             [](const Packet& p) { return p.kind == Protocol::Kind::Frame; });
            if (oldest != m_state->queue.end()) {
                oldest->buffer->forget();
                m_state->queue.erase(oldest);
                ++m_state->counts.dropped;
                continue;
            }
        }
        m_state->changed.wait(lock);
    }
    packet.sequence = ++m_state->submitted;
    m_state->queue.push_back(std::move(packet));
    lock.unlock();
    m_state->changed.notify_all();
    m_lastError.clear();
    return true;
}

bool StreamClient::registerSchema(const FrameSchema& schema) {
    if (backgroundFailure()) return false;
    std::string why;
    if (!Protocol::validateSchema(schema, why)) return fail(why);
    m_schema = schema;
    return sendSchema();
}
bool StreamClient::sendSchema() {
    Protocol::Header header;
    header.kind = Protocol::Kind::Schema;
    header.mesh_revision = ++m_schemaRevision;
    const std::string payload = Protocol::encodeSchema(*m_schema);
    Packet packet = makePacket(header, payload.size());
    std::memcpy(packet.buffer->data.get() + Protocol::HeaderBytes, payload.data(), payload.size());
    return submit(std::move(packet));
}
bool StreamClient::sendMesh(uint64_t mesh_id, std::string_view mesh_text) {
    if (backgroundFailure()) return false;
    if (!mesh_text.starts_with("MFEM mesh v") && !mesh_text.starts_with("MFEM NURBS mesh v") && !mesh_text.starts_with("MFEM NC mesh v"))
        return fail("Mesh text must be MFEM mesh text (MFEM mesh, NURBS mesh or NC mesh)");
    if (mesh_text.size() > Protocol::MaxPayloadBytes) return fail("Packet exceeds 64 MiB payload limit");
    if (!reachable(Protocol::Kind::Mesh, mesh_id)) return false;
    if (m_schema && !sendSchema()) return false;
    Protocol::Header header;
    header.kind = Protocol::Kind::Mesh;
    header.mesh_id = mesh_id;
    header.mesh_revision = ++m_nextRevision;
    Packet packet = makePacket(header, mesh_text.size());
    std::memcpy(packet.buffer->data.get() + Protocol::HeaderBytes, mesh_text.data(), mesh_text.size());
    if (!submit(std::move(packet))) return false;
    m_meshes[mesh_id] = {header.mesh_revision, 0, false};
    return true;
}
bool StreamClient::sendFrame(uint64_t mesh_id, std::span<const FieldData> fields, const FrameMetadata& metadata,
                             const FieldData* displacement) {
    if (backgroundFailure()) return false;
    auto found = m_meshes.find(mesh_id);
    if (found == m_meshes.end()) return fail("Call sendMesh before sendFrame");
    Registration& registration = found->second;
    // Without a queue, a refusal for the connection comes before checking the frame.
    if (!reachable(Protocol::Kind::Frame, mesh_id)) return false;
    if (registration.has_frame && metadata.frame_index <= registration.last_frame)
        return fail("Frame indexes must increase within a mesh registration");
    if (!std::isfinite(metadata.sim_time)) return fail("Frame time must be finite");
    if (fields.empty() || fields.size() > Protocol::MaxFields) return fail("A frame needs between 1 and 64 fields");
    std::string descriptor;
    const bool described = !metadata.values.empty() || !metadata.tags.empty();
    if (described) {
        if (!m_schema) return fail("Register a schema before sending frame values or tags");
        std::string why;
        if (!Protocol::encodeDescriptor(*m_schema, metadata, descriptor, why)) return fail(why);
    }
    // Check everything and size the payload, then write it once. Binary values
    // are checked as they are written; text is formatted here.
    std::vector<std::string> texts;
    auto check = [&](const FieldData& field) -> bool {
        if (!validSpaceHeader(field.space)) return fail("Field " + std::string(field.name) + " needs a FiniteElementSpace header");
        if (field.values.empty()) return fail("Field " + std::string(field.name) + " has no coefficients");
        if (!m_binaryFields) {
            if (!finite(field.values)) return fail("Field " + std::string(field.name) + " has non-finite coefficients");
            texts.push_back(encodeText(field));
        }
        return true;
    };
    auto dataSize = [&](const FieldData& field, size_t index) { return m_binaryFields ? binarySize(field) : texts[index].size(); };
    std::unordered_set<std::string_view> names;
    size_t payload = 0;
    for (size_t i = 0; i < fields.size(); ++i) {
        const auto& field = fields[i];
        if (field.name.empty() || field.name.size() > Protocol::MaxNameBytes || field.name.find('\0') != std::string_view::npos ||
            !names.insert(field.name).second)
            return fail("Field names must be unique, nonempty, and at most 255 bytes");
        if (!check(field)) return false;
        payload += 12 + field.name.size() + dataSize(field, i);
        if (payload > Protocol::MaxPayloadBytes) return fail("Frame exceeds 64 MiB payload limit");
    }
    if (displacement) {
        if (!check(*displacement)) return false;
        payload += 8 + dataSize(*displacement, fields.size());
    }
    payload += descriptor.size();
    if (payload > Protocol::MaxPayloadBytes) return fail("Frame exceeds 64 MiB payload limit");

    Protocol::Header header;
    header.kind = Protocol::Kind::Frame;
    header.mesh_id = mesh_id;
    header.mesh_revision = registration.revision;
    header.metadata = metadata;
    header.field_count = static_cast<uint32_t>(fields.size());
    header.flags = (displacement ? Protocol::HasDisplacement : 0) | (m_binaryFields ? Protocol::BinaryFields : 0) |
                   (described ? Protocol::HasDescriptor : 0);
    Packet packet = makePacket(header, payload);
    Writer out{packet.buffer->data.get() + Protocol::HeaderBytes};
    auto write = [&](const FieldData& field, size_t index) {
        if (!m_binaryFields) { out.bytes(texts[index]); return true; }
        if (writeBinary(out, field)) return true;
        return fail("Field " + std::string(field.name) + " has non-finite coefficients");
    };
    for (size_t i = 0; i < fields.size(); ++i) {
        out.integer(fields[i].name.size(), 4);
        out.integer(dataSize(fields[i], i), 8);
        out.bytes(fields[i].name);
        if (!write(fields[i], i)) return false;
    }
    if (displacement) {
        out.integer(dataSize(*displacement, fields.size()), 8);
        if (!write(*displacement, fields.size())) return false;
    }
    out.bytes(descriptor);
    if (!submit(std::move(packet))) return false;
    registration.last_frame = metadata.frame_index;
    registration.has_frame = true;
    return true;
}
} // namespace RayFEM
