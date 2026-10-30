#include "libRayfem/network/stream.hpp"
#include "libRayfem/network/zmq.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <stdexcept>
#include <unordered_set>

namespace RayFEM {
namespace {
bool validSpaceHeader(std::string_view space) {
    return (space.starts_with("FiniteElementSpace\n") || space.starts_with("MFEM FiniteElementSpace v1.0\n")) &&
           space.size() <= Protocol::MaxSpaceHeaderBytes && space.find('\0') == std::string_view::npos;
}
bool finite(std::span<const double> values) {
    for (double v : values) if (!std::isfinite(v)) return false;
    return true;
}
// "RFB1", the header length and text, the value count, then IEEE doubles.
std::string encodeBinary(const FieldData& field) {
    std::string out(Protocol::BinaryFieldMagic);
    out.reserve(4 + 4 + field.space.size() + 8 + 8*field.values.size());
    Protocol::appendInteger(out, field.space.size(), 4);
    out += field.space;
    Protocol::appendInteger(out, field.values.size(), 8);
    for (double value : field.values) Protocol::appendInteger(out, std::bit_cast<uint64_t>(value), 8);
    return out;
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

// The PUSH socket, and libzmq's monitor of its connection to the viewer.
// Streaming is one-way, so the connection is the only feedback: a send with
// no connected viewer is refused instead of queued, and a reconnection (the
// viewer's listener closed and opened again, for example after changing its
// host on the same port) starts a new epoch whose listener has none of the
// meshes sent before it.
struct StreamClient::Socket {
    // The first member: the host's signal handlers are kept before CZMQ starts.
    bool keep_signals = (KeepHostSignals(), true);
    SocketPtr value{zsock_new(ZMQ_PUSH)};
    SocketPtr monitor;
    std::string address;
    bool connected = false;
    uint64_t epoch = 0;  // connections made so far

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
};
StreamClient::~StreamClient() = default;
StreamClient::StreamClient(StreamClient&&) noexcept = default;
StreamClient& StreamClient::operator=(StreamClient&&) noexcept = default;
StreamClient::StreamClient(const std::string& host, size_t port) : m_socket(std::make_unique<Socket>()) {
    if (!m_socket->value || port == 0 || port > 65535) throw std::runtime_error("Invalid RayFEM client endpoint");
    zsock_set_linger(m_socket->value, 1000);
    zsock_set_sndtimeo(m_socket->value, 1000);
    zsock_set_sndhwm(m_socket->value, 64);
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
    m_socket->poll(0);
    return m_socket->connected;
}
const std::string& StreamClient::address() const { return m_socket->address; }
bool StreamClient::reachable() {
    if (!m_socket->monitor) return true;
    m_socket->poll(m_socket->connected ? 0 : 1000);
    if (m_socket->connected) return true;
    return fail("No viewer is listening on " + m_socket->address +
                "; it may have moved to another host or port, or closed. Nothing was sent.");
}
bool StreamClient::fail(std::string message) { m_lastError = std::move(message); return false; }
bool StreamClient::sendPacket(Protocol::Header header, const std::string& payload) {
    if (payload.size() > Protocol::MaxPayloadBytes) return fail("Packet exceeds 64 MiB payload limit");
    if (!reachable()) return false;
    header.payload_bytes = payload.size();
    header.session_id = m_sessionId;
    std::string bytes = Protocol::encodeHeader(header);
    bytes += payload;
    FramePtr packet(zframe_new(bytes.data(), bytes.size()));
    zframe_t* raw = packet.release();
    const int result = zframe_send(&raw, m_socket->value, 0);
    if (raw) zframe_destroy(&raw);
    if (result != 0) return fail("Timed out sending RayFEM packet");
    m_lastError.clear();
    return true;
}
bool StreamClient::registerSchema(const FrameSchema& schema) {
    std::string why;
    if (!Protocol::validateSchema(schema, why)) return fail(why);
    m_schema = schema;
    return sendSchema();
}
bool StreamClient::sendSchema() {
    Protocol::Header header;
    header.kind = Protocol::Kind::Schema;
    header.mesh_revision = ++m_schemaRevision;
    return sendPacket(header, Protocol::encodeSchema(*m_schema));
}
bool StreamClient::sendMesh(uint64_t mesh_id, std::string_view mesh_text) {
    if (!mesh_text.starts_with("MFEM mesh v") && !mesh_text.starts_with("MFEM NURBS mesh v") && !mesh_text.starts_with("MFEM NC mesh v"))
        return fail("Mesh text must be MFEM mesh text (MFEM mesh, NURBS mesh or NC mesh)");
    if (m_schema && !sendSchema()) return false;
    Protocol::Header header;
    header.kind = Protocol::Kind::Mesh;
    header.mesh_id = mesh_id;
    header.mesh_revision = ++m_nextRevision;
    if (!sendPacket(header, std::string(mesh_text))) return false;
    m_meshes[mesh_id] = {header.mesh_revision, 0, false, m_socket->epoch};
    return true;
}
bool StreamClient::sendFrame(uint64_t mesh_id, std::span<const FieldData> fields, const FrameMetadata& metadata,
                             const FieldData* displacement) {
    auto found = m_meshes.find(mesh_id);
    if (found == m_meshes.end()) return fail("Call sendMesh before sendFrame");
    Registration& registration = found->second;
    if (!reachable()) return false;
    if (m_socket->monitor && registration.epoch != m_socket->epoch)
        return fail("The viewer on " + m_socket->address + " stopped listening and started again since mesh " +
                    std::to_string(mesh_id) + " was sent (it may have changed its host or port), so it no longer "
                    "has the mesh. Send the mesh again; nothing was sent.");
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
    auto check = [&](const FieldData& field) -> bool {
        if (!validSpaceHeader(field.space)) return fail("Field " + std::string(field.name) + " needs a FiniteElementSpace header");
        if (field.values.empty()) return fail("Field " + std::string(field.name) + " has no coefficients");
        if (!finite(field.values)) return fail("Field " + std::string(field.name) + " has non-finite coefficients");
        return true;
    };
    std::unordered_set<std::string_view> names;
    std::string payload;
    for (const auto& field : fields) {
        if (field.name.empty() || field.name.size() > Protocol::MaxNameBytes || field.name.find('\0') != std::string_view::npos ||
            !names.insert(field.name).second)
            return fail("Field names must be unique, nonempty, and at most 255 bytes");
        if (!check(field)) return false;
        const std::string data = m_binaryFields ? encodeBinary(field) : encodeText(field);
        if (payload.size() + 12 + field.name.size() + data.size() > Protocol::MaxPayloadBytes)
            return fail("Frame exceeds 64 MiB payload limit");
        Protocol::appendInteger(payload, field.name.size(), 4);
        Protocol::appendInteger(payload, data.size(), 8);
        payload += field.name;
        payload += data;
    }
    if (displacement) {
        if (!check(*displacement)) return false;
        const std::string data = m_binaryFields ? encodeBinary(*displacement) : encodeText(*displacement);
        if (payload.size() + 8 + data.size() > Protocol::MaxPayloadBytes) return fail("Frame exceeds 64 MiB payload limit");
        Protocol::appendInteger(payload, data.size(), 8);
        payload += data;
    }
    if (described) {
        if (payload.size() + descriptor.size() > Protocol::MaxPayloadBytes) return fail("Frame exceeds 64 MiB payload limit");
        payload += descriptor;
    }
    Protocol::Header header;
    header.kind = Protocol::Kind::Frame;
    header.mesh_id = mesh_id;
    header.mesh_revision = registration.revision;
    header.metadata = metadata;
    header.field_count = static_cast<uint32_t>(fields.size());
    header.flags = (displacement ? Protocol::HasDisplacement : 0) | (m_binaryFields ? Protocol::BinaryFields : 0) |
                   (described ? Protocol::HasDescriptor : 0);
    if (!sendPacket(header, payload)) return false;
    registration.last_frame = metadata.frame_index;
    registration.has_frame = true;
    return true;
}
} // namespace RayFEM
