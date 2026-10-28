#include "libRayfem/network/client.hpp"
#include "libRayfem/network/zmq.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <random>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace RayFEM {
namespace {
std::string saveField(const mfem::GridFunction& gf) {
    std::ostringstream stream;
    stream << std::setprecision(17);
    gf.Save(stream);
    return stream.str();
}
bool finiteField(const mfem::GridFunction& gf) {
    for (int i = 0; i < gf.Size(); ++i) if (!std::isfinite(gf(i))) return false;
    return true;
}
}
struct Client::Socket {
    SocketPtr value{zsock_new(ZMQ_PUSH)};
};
Client::~Client() = default;
Client::Client(Client&&) noexcept = default;
Client& Client::operator=(Client&&) noexcept = default;
Client::Client(const std::string& host, size_t port) : m_socket(std::make_unique<Socket>()) {
    if (!m_socket->value || port == 0 || port > 65535) throw std::runtime_error("Invalid RayFEM client endpoint");
    zsock_set_linger(m_socket->value, 1000);
    zsock_set_sndtimeo(m_socket->value, 1000);
    zsock_set_sndhwm(m_socket->value, 64);
    const std::string address = "tcp://" + host + ":" + std::to_string(port);
    if (zsock_connect(m_socket->value, "%s", address.c_str()) != 0)
        throw std::runtime_error("Cannot connect RayFEM client to " + address);
    std::random_device random;
    m_sessionId = (static_cast<uint64_t>(random()) << 32) ^ random()
        ^ static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
    if (!m_sessionId) m_sessionId = 1;
}
bool Client::fail(std::string message) { m_lastError = std::move(message); return false; }
bool Client::sendPacket(Protocol::Header header, const std::string& payload) {
    if (payload.size() > Protocol::MaxPayloadBytes) return fail("Packet exceeds 64 MiB payload limit");
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
bool Client::sendMesh(uint64_t mesh_id, mfem::Mesh& mesh) {
    if (mesh.GetNE() == 0 || mesh.Dimension() < 1 || mesh.Dimension() > 3 || mesh.SpaceDimension() > 3)
        return fail("Only nonempty meshes in one to three dimensions are supported");
    std::ostringstream stream;
    stream << std::setprecision(17);
    mesh.Print(stream);
    Protocol::Header header;
    header.kind = Protocol::Kind::Mesh;
    header.mesh_id = mesh_id;
    header.mesh_revision = ++m_nextRevision;
    if (!sendPacket(header, stream.str())) return false;
    m_meshes[mesh_id] = {&mesh, header.mesh_revision, mesh.GetSequence(), mesh.GetNV(), mesh.GetNE(), 0, false};
    return true;
}
bool Client::sendFrame(uint64_t mesh_id, const std::vector<NamedField>& fields,
                       const FrameMetadata& metadata, mfem::GridFunction* displacement) {
    auto found = m_meshes.find(mesh_id);
    if (found == m_meshes.end()) return fail("Call sendMesh before sendFrame");
    Registration& registration = found->second;
    auto* mesh = registration.mesh;
    if (mesh->GetSequence() != registration.sequence || mesh->GetNV() != registration.vertices || mesh->GetNE() != registration.elements)
        return fail("Mesh topology changed; register the mesh again");
    if (registration.has_frame && metadata.frame_index <= registration.last_frame)
        return fail("Frame indexes must increase within a mesh registration");
    if (!std::isfinite(metadata.sim_time)) return fail("Frame time must be finite");
    if (fields.empty() || fields.size() > Protocol::MaxFields) return fail("A frame needs between 1 and 64 fields");
    auto valid = [&](mfem::GridFunction* gf) {
        return gf && gf->FESpace() && gf->FESpace()->GetMesh() == mesh &&
               gf->Size() == gf->FESpace()->GetVSize() && finiteField(*gf);
    };
    std::unordered_set<std::string> names;
    std::string payload;
    for (const auto& field : fields) {
        if (field.name.empty() || field.name.size() > Protocol::MaxNameBytes || field.name.find('\0') != std::string::npos || !names.insert(field.name).second)
            return fail("Field names must be unique, nonempty, and at most 255 bytes");
        if (!valid(field.gf)) return fail("Field is invalid, non-finite, or belongs to a different mesh");
        std::string data = saveField(*field.gf);
        if (payload.size() + 12 + field.name.size() + data.size() > Protocol::MaxPayloadBytes)
            return fail("Frame exceeds 64 MiB payload limit");
        Protocol::appendInteger(payload, field.name.size(), 4);
        Protocol::appendInteger(payload, data.size(), 8);
        payload += field.name;
        payload += data;
    }
    if (displacement) {
        if (!valid(displacement) || displacement->FESpace()->GetVDim() != mesh->SpaceDimension() ||
            displacement->FESpace()->FEColl()->GetRangeType(mesh->Dimension()) != mfem::FiniteElement::SCALAR)
            return fail("Displacement must be a pointwise vector field on the registered mesh");
        std::string data = saveField(*displacement);
        if (payload.size() + 8 + data.size() > Protocol::MaxPayloadBytes) return fail("Frame exceeds 64 MiB payload limit");
        Protocol::appendInteger(payload, data.size(), 8);
        payload += data;
    }
    Protocol::Header header;
    header.kind = Protocol::Kind::Frame;
    header.mesh_id = mesh_id;
    header.mesh_revision = registration.revision;
    header.metadata = metadata;
    header.field_count = static_cast<uint32_t>(fields.size());
    header.flags = displacement ? Protocol::HasDisplacement : 0;
    if (!sendPacket(header, payload)) return false;
    registration.last_frame = metadata.frame_index;
    registration.has_frame = true;
    return true;
}
bool Client::send(mfem::Mesh& mesh, mfem::GridFunction& gf, const FrameMetadata& metadata) {
    return sendMesh(0, mesh) && sendFrame(0, {{"solution", &gf}}, metadata);
}
} // namespace RayFEM
