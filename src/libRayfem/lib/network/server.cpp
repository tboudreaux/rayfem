#include "libRayfem/network/server.hpp"
#include "libRayfem/network/zmq.hpp"
#include "../mesh/input_validation.hpp"
#include "libRayfem/threading.hpp"

#include <algorithm>
#include <bit>
#include <cctype>
#include <charconv>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <unordered_set>

namespace RayFEM {
namespace {
constexpr size_t MaxQueuedFrames = 64;
constexpr size_t MaxQueuedBytes = 128 * 1024 * 1024;
constexpr size_t MaxCachedMeshes = 16;
constexpr size_t MaxCachedMeshBytes = 128 * 1024 * 1024;
struct SerializedMesh {
    Protocol::Header header;
    std::string data;
    uint64_t last_use = 0;
    uint32_t last_frame = 0;
    bool has_frame = false;
    // Accessed only by the fetchFrames caller, never by the receiver.
    std::weak_ptr<mfem::Mesh> decoded;
    bool decode_failed = false;
    size_t estimated_bytes = 0;
};
struct SerializedFrame {
    Protocol::Header header;
    std::shared_ptr<SerializedMesh> mesh;
    std::vector<std::pair<std::string, std::string>> fields;
    std::string displacement;
    size_t queued_bytes = 0;
};
std::string meshKey(const Protocol::Header& header) {
    return std::to_string(header.session_id) + ":" + std::to_string(header.mesh_id);
}
bool readBlob(std::string_view payload, size_t& offset, size_t bytes, std::string& target) {
    if (offset > payload.size() || bytes > payload.size() - offset) return false;
    target.assign(payload.data() + offset, bytes);
    offset += bytes;
    return true;
}
bool parseFrame(std::string_view payload, SerializedFrame& frame, std::string& error) {
    size_t pos = 0;
    std::unordered_set<std::string> names;
    for (uint32_t i = 0; i < frame.header.field_count; ++i) {
        uint64_t name_size = 0, data_size = 0;
        std::string name, data;
        if (!Protocol::readInteger(payload, pos, 4, name_size) ||
            !Protocol::readInteger(payload, pos, 8, data_size) ||
            !name_size || name_size > Protocol::MaxNameBytes || !data_size ||
            !readBlob(payload, pos, name_size, name) || !readBlob(payload, pos, data_size, data)) {
            error = "Truncated or invalid frame field"; return false;
        }
        if (name.find('\0') != std::string::npos || !names.insert(name).second) {
            error = "Invalid or duplicate field name"; return false;
        }
        frame.fields.emplace_back(std::move(name), std::move(data));
    }
    if (frame.header.flags & Protocol::HasDisplacement) {
        uint64_t bytes = 0;
        if (!Protocol::readInteger(payload, pos, 8, bytes) || !bytes || !readBlob(payload, pos, bytes, frame.displacement)) {
            error = "Truncated displacement field"; return false;
        }
    }
    if (pos != payload.size()) { error = "Unexpected bytes after frame payload"; return false; }
    return true;
}

size_t estimateFieldBytes(const mfem::GridFunction& field, size_t serialized_bytes) {
    const auto& space = *field.FESpace();
    size_t element_dofs = 0, maximum_element_dofs = 0;
    for (int e = 0; e < space.GetNE(); ++e) {
        const size_t dofs = space.GetFE(e)->GetDof();
        element_dofs += dofs;
        maximum_element_dofs = std::max(maximum_element_dofs, dofs);
    }
    // Allow for coefficient storage, global/local DOF maps, connectivity, and
    // dense local FE workspaces. Serialized size provides an additional proxy
    // for collection-specific bookkeeping without depending on MFEM internals.
    return serialized_bytes + static_cast<size_t>(field.Size()) * sizeof(mfem::real_t)
        + static_cast<size_t>(space.GetNDofs()) * 8 * sizeof(int)
        + element_dofs * 2 * sizeof(int) + static_cast<size_t>(space.GetNE()) * 64
        + maximum_element_dofs * maximum_element_dofs * 2 * sizeof(mfem::real_t);
}

size_t estimateMeshBytes(const mfem::Mesh& mesh, size_t serialized_bytes) {
    size_t bytes = serialized_bytes * 2 + static_cast<size_t>(mesh.GetNV()) * 3 * sizeof(mfem::real_t)
        + static_cast<size_t>(mesh.GetNE()) * 512 + static_cast<size_t>(mesh.GetNBE()) * 128
        + static_cast<size_t>(mesh.GetNEdges()) * 64 + static_cast<size_t>(mesh.GetNumFaces()) * 128;
    if (mesh.GetNodes()) bytes += estimateFieldBytes(*mesh.GetNodes(), 0);
    return bytes;
}

using detail::validateCollectionOrder;
using detail::validateMeshText;
using detail::MaxNURBSOrder;

// MFEM's text loader does not validate VDim/Ordering before allocating. Validate
// those values before loading a space from text.
void validateSpaceHeader(mfem::Mesh& mesh, std::string_view text) {
    const std::string data(text);
    std::istringstream preflight(data);
    std::string format, collection_line, dimension_line, ordering_line;
    std::getline(preflight, format);
    std::getline(preflight, collection_line);
    std::getline(preflight, dimension_line);
    std::getline(preflight, ordering_line);
    if (format != "FiniteElementSpace" && format != "MFEM FiniteElementSpace v1.0")
        throw std::runtime_error("Invalid finite element space header");
    if (!collection_line.starts_with("FiniteElementCollection: ") || collection_line.size() > 160)
        throw std::runtime_error("Invalid finite element collection");
    auto readProperty = [](const std::string& line, const char* label) {
        std::istringstream input(line);
        std::string name, extra;
        int result = -1;
        if (!(input >> name >> result) || name != label || (input >> extra))
            throw std::runtime_error("Invalid finite element space property");
        return result;
    };
    const int vdim = readProperty(dimension_line, "VDim:");
    const int ordering = readProperty(ordering_line, "Ordering:");
    if (vdim < 1 || vdim > 64 || (ordering != 0 && ordering != 1))
        throw std::runtime_error("Unsupported field dimension or ordering");
    validateCollectionOrder(std::string_view(collection_line).substr(25));
    std::string section;
    while (std::getline(preflight, section)) {
        if (section == "NURBS_order" || section == "NURBS_orders") {
            if (!mesh.NURBSext) throw std::runtime_error("NURBS field requires a NURBS mesh");
            const int count = section == "NURBS_order" ? 1 : mesh.NURBSext->GetNKV();
            for (int i = 0; i < count; ++i) {
                int order = 0;
                if (!(preflight >> order) || order < 0 || order > MaxNURBSOrder)
                    throw std::runtime_error("Invalid or excessive NURBS field order");
            }
        }
    }
}

// A decoded space shared by every field of a mesh registration with the same
// header, so each frame does not rebuild identical DOF tables.
struct DecodedSpace {
    std::shared_ptr<mfem::Mesh> mesh;
    std::string header;
    std::unique_ptr<mfem::FiniteElementCollection> collection;
    std::unique_ptr<mfem::FiniteElementSpace> space;
};
struct SpaceHolder { std::shared_ptr<DecodedSpace> shared; };
// Keeps its space alive; the base order destroys the field before the space.
class DecodedField : private SpaceHolder, public mfem::GridFunction {
public:
    explicit DecodedField(std::shared_ptr<DecodedSpace> space)
        : SpaceHolder{std::move(space)}, mfem::GridFunction(shared->space.get()) {}
};
using SpaceCache = std::vector<std::weak_ptr<DecodedSpace>>;

std::shared_ptr<DecodedSpace> cachedSpace(SpaceCache& cache, const std::shared_ptr<mfem::Mesh>& mesh,
                                          std::string_view data, bool exact) {
    std::erase_if(cache, [](const auto& entry) { return entry.expired(); });
    for (const auto& entry : cache) {
        auto space = entry.lock();
        if (space && space->mesh == mesh && (exact ? data == space->header : data.starts_with(space->header))) return space;
    }
    return nullptr;
}

std::shared_ptr<DecodedSpace> loadSpace(SpaceCache& cache, const std::shared_ptr<mfem::Mesh>& mesh,
                                        std::string header) {
    validateSpaceHeader(*mesh, header);
    auto result = std::make_shared<DecodedSpace>();
    result->mesh = mesh;
    std::istringstream stream(header);
    result->space = std::make_unique<mfem::FiniteElementSpace>();
    result->collection.reset(result->space->Load(mesh.get(), stream));
    if (stream.bad() || !result->collection || result->space->GetVSize() < 1)
        throw std::runtime_error("Truncated or invalid finite element space");
    result->header = std::move(header);
    cache.push_back(result);
    return result;
}

void readValues(std::string_view text, mfem::GridFunction& field) {
    const char* cursor = text.data();
    const char* const end = text.data() + text.size();
    auto skip = [&] { while (cursor < end && std::isspace(static_cast<unsigned char>(*cursor))) ++cursor; };
    for (int i = 0; i < field.Size(); ++i) {
        skip();
        double value = 0;
        const auto parsed = std::from_chars(cursor, end, value);
        if (parsed.ec != std::errc{} || !std::isfinite(value))
            throw std::runtime_error("Truncated or non-finite grid function");
        field(i) = value;
        cursor = parsed.ptr;
    }
    skip();
    if (cursor != end) throw std::runtime_error("Unexpected data after grid function");
}

struct DecodedFieldResult { std::unique_ptr<mfem::GridFunction> field; bool shared_space = false; };

// MFEM text (GridFunction::Save) or the binary layout in packet.hpp.
DecodedFieldResult decodeField(const std::shared_ptr<mfem::Mesh>& mesh, const std::string& data,
                               bool binary, SpaceCache& cache) {
    DecodedFieldResult result;
    std::shared_ptr<DecodedSpace> space;
    std::string_view values;
    if (binary) {
        const std::string_view bytes(data);
        size_t pos = Protocol::BinaryFieldMagic.size();
        uint64_t header_size = 0, count = 0;
        if (!bytes.starts_with(Protocol::BinaryFieldMagic) || !Protocol::readInteger(bytes, pos, 4, header_size) ||
            header_size == 0 || header_size > Protocol::MaxSpaceHeaderBytes || header_size > bytes.size() - pos)
            throw std::runtime_error("Invalid binary field header");
        const auto header = bytes.substr(pos, header_size);
        pos += header_size;
        if (!Protocol::readInteger(bytes, pos, 8, count) || count != (bytes.size() - pos)/8 || (bytes.size() - pos)%8)
            throw std::runtime_error("Truncated binary field values");
        space = cachedSpace(cache, mesh, header, true);
        result.shared_space = bool(space);
        if (!space) space = loadSpace(cache, mesh, std::string(header));
        if (count != static_cast<uint64_t>(space->space->GetVSize()))
            throw std::runtime_error("Binary field size does not match its finite element space");
        auto field = std::make_unique<DecodedField>(space);
        for (int i = 0; i < field->Size(); ++i) {
            uint64_t bits = 0;
            Protocol::readInteger(bytes, pos, 8, bits);
            const double value = std::bit_cast<double>(bits);
            if (!std::isfinite(value)) throw std::runtime_error("Non-finite grid function value");
            (*field)(i) = value;
        }
        result.field = std::move(field);
        return result;
    }
    space = cachedSpace(cache, mesh, data, false);
    result.shared_space = bool(space);
    if (!space) {
        // Find where MFEM's loader stops reading the space, then cache that
        // exact header text for later frames.
        validateSpaceHeader(*mesh, data);
        std::istringstream stream(data);
        mfem::FiniteElementSpace probe;
        std::unique_ptr<mfem::FiniteElementCollection> collection(probe.Load(mesh.get(), stream));
        const auto end = stream.tellg();
        if (stream.fail() || end <= 0 || probe.GetVSize() < 1 || static_cast<size_t>(probe.GetVSize()) > data.size())
            throw std::runtime_error("Truncated or invalid finite element space");
        space = loadSpace(cache, mesh, data.substr(0, static_cast<size_t>(end)));
    }
    values = std::string_view(data).substr(space->header.size());
    auto field = std::make_unique<DecodedField>(space);
    readValues(values, *field);
    result.field = std::move(field);
    return result;
}
} // namespace

struct Server::Impl {
    std::string address, control_address;
    size_t stream_port;
    std::atomic<bool> running{true};
    std::atomic<bool> is_ready{false};
    std::atomic<bool> viewer_ready{false};
    std::atomic<bool> shutdown_requested{false};
    std::atomic<size_t> received{0}, dropped{0}, errors{0}, skipped{0};
    std::atomic<bool> receiving{true};
    mutable std::mutex mutex;
    std::string last_error;
    std::deque<SerializedFrame> queue;
    SpaceCache spaces;  // fetchFrames caller only
    size_t queued_bytes = 0;
    std::thread worker;

    Impl(std::string endpoint, std::string control_endpoint, size_t port)
        : address(std::move(endpoint)), control_address(std::move(control_endpoint)),
          stream_port(port), worker(&Impl::networkLoop, this) {}
    ~Impl() { running = false; if (worker.joinable()) worker.join(); }
    void error(std::string message) {
        ++errors;
        std::lock_guard lock(mutex);
        last_error = std::move(message);
    }
    void answerControl(zsock_t* socket) {
        FramePtr request(zframe_recv(socket));
        if (!request) return;
        bool multipart = zsock_rcvmore(socket);
        while (running && zsock_rcvmore(socket)) {
            FramePtr ignored(zframe_recv(socket));
            if (!ignored) return;
        }
        std::string response = "RAYFEM_INVALID 1";
        bool shutdown = false;
        if (!multipart && zframe_size(request) <= 256) {
            std::istringstream input(std::string(reinterpret_cast<const char*>(zframe_data(request)), zframe_size(request)));
            std::string command, nonce, extra;
            int version = 0;
            size_t requested_port = 0;
            if ((input >> command >> version >> requested_port >> nonce) && !(input >> extra) &&
                (command == "RAYFEM_PROBE" || command == "RAYFEM_SHUTDOWN") &&
                version == 1 && requested_port == stream_port &&
                !nonce.empty() && nonce.size() <= 64 &&
                std::all_of(nonce.begin(), nonce.end(), [](unsigned char c) {
                    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
                })) {
                shutdown = command == "RAYFEM_SHUTDOWN";
                response = std::string(shutdown ? "RAYFEM_STOPPING 1 " :
                    (viewer_ready ? "RAYFEM_READY 1 " : "RAYFEM_STARTING 1 "))
                    + std::to_string(stream_port) + " " + nonce;
            }
        }
        if (zstr_send(socket, response.c_str()) == 0 && shutdown) shutdown_requested = true;
    }
    void networkLoop() {
        // Close the control endpoint last: its disconnect confirms that the
        // streaming receiver and queued network state have been released.
        SocketPtr control;
        SocketPtr pull(zsock_new(ZMQ_PULL));
        if (!pull) { error("Cannot create receiver socket"); return; }
        zsock_set_ipv6(pull, 1);
        zsock_set_linger(pull, 0);
        zsock_set_rcvtimeo(pull, 100);
        zsock_set_rcvhwm(pull, 64);
        zsock_set_maxmsgsize(pull, Protocol::HeaderBytes + Protocol::MaxPayloadBytes);
        if (zsock_bind(pull, "%s", address.c_str()) < 0) {
            error("Cannot bind receiver to " + address); return;
        }
        if (!control_address.empty()) {
            control = SocketPtr(zsock_new(ZMQ_REP));
            if (!control) { error("Cannot create viewer control socket"); return; }
            zsock_set_ipv6(control, 1);
            // Flush the shutdown acknowledgement before the main thread's
            // normal destruction closes this connection.
            zsock_set_linger(control, 100);
            zsock_set_rcvtimeo(control, 100);
            zsock_set_sndtimeo(control, 100);
            zsock_set_rcvhwm(control, 16);
            zsock_set_maxmsgsize(control, 256);
            if (zsock_bind(control, "%s", control_address.c_str()) < 0) {
                error("Cannot bind viewer control endpoint to " + control_address); return;
            }
        }
        is_ready = true;
        std::unordered_map<std::string, std::shared_ptr<SerializedMesh>> meshes;
        size_t mesh_bytes = 0;
        uint64_t use_sequence = 0;
        while (running) {
            zmq_pollitem_t sockets[] = {
                {zsock_resolve(pull), 0, ZMQ_POLLIN, 0},
                {control ? zsock_resolve(control) : nullptr, 0, ZMQ_POLLIN, 0}
            };
            if (zmq_poll(sockets, control ? 2 : 1, 100) <= 0) continue;
            if (control && (sockets[1].revents & ZMQ_POLLIN)) answerControl(control);
            if (!(sockets[0].revents & ZMQ_POLLIN)) continue;
            FramePtr packet(zframe_recv(pull));
            if (!packet) continue;
            if (zsock_rcvmore(pull)) {
                while (running && zsock_rcvmore(pull)) { FramePtr ignored(zframe_recv(pull)); if (!ignored) break; }
                error("Multipart messages are not valid RayFEM v2 packets");
                continue;
            }
            try {
                std::string_view bytes(reinterpret_cast<const char*>(zframe_data(packet)), zframe_size(packet));
                Protocol::Header header;
                std::string why;
                if (!Protocol::decodeHeader(bytes, header, why)) { error(why); continue; }
                const auto payload = bytes.substr(Protocol::HeaderBytes);
                const std::string key = meshKey(header);
                auto found = meshes.find(key);
                if (header.kind == Protocol::Kind::Mesh) {
                    if (found != meshes.end() && header.mesh_revision <= found->second->header.mesh_revision) {
                        error("Stale mesh registration"); continue;
                    }
                    if (!payload.starts_with("MFEM mesh v") && !payload.starts_with("MFEM NURBS mesh v") && !payload.starts_with("MFEM NC mesh v")) {
                        error("Invalid MFEM mesh header"); continue;
                    }
                    if (found != meshes.end()) { mesh_bytes -= found->second->data.size(); meshes.erase(found); }
                    while (!meshes.empty() && (meshes.size() >= MaxCachedMeshes || mesh_bytes + payload.size() > MaxCachedMeshBytes)) {
                        auto oldest = std::min_element(meshes.begin(), meshes.end(), [](const auto& a, const auto& b) { return a.second->last_use < b.second->last_use; });
                        mesh_bytes -= oldest->second->data.size();
                        meshes.erase(oldest);
                    }
                    auto mesh = std::make_shared<SerializedMesh>();
                    mesh->header = header;
                    mesh->data = payload;
                    mesh->last_use = ++use_sequence;
                    mesh_bytes += mesh->data.size();
                    meshes.emplace(key, std::move(mesh));
                    continue;
                }
                if (found == meshes.end() || found->second->header.mesh_revision != header.mesh_revision) {
                    error("Frame references an unknown or stale mesh; sendMesh is required"); continue;
                }
                auto& mesh = found->second;
                if (mesh->has_frame && header.metadata.frame_index <= mesh->last_frame) {
                    error("Stale or duplicate frame index"); continue;
                }
                if (!receiving) { ++skipped; continue; }
                SerializedFrame frame;
                frame.header = header;
                frame.mesh = mesh;
                if (!parseFrame(payload, frame, why)) { error(why); continue; }
                mesh->last_use = ++use_sequence;
                mesh->has_frame = true;
                mesh->last_frame = header.metadata.frame_index;
                frame.queued_bytes = payload.size() + mesh->data.size();
                std::lock_guard lock(mutex);
                while (!queue.empty() && (queue.size() >= MaxQueuedFrames || queued_bytes + frame.queued_bytes > MaxQueuedBytes)) {
                    queued_bytes -= queue.front().queued_bytes;
                    queue.pop_front();
                    ++dropped;
                }
                queued_bytes += frame.queued_bytes;
                queue.push_back(std::move(frame));
            } catch (const std::exception& exception) { error(std::string("Packet rejected: ") + exception.what()); }
        }
        is_ready = false;
    }
};

Server::Server(const std::string& listen, size_t port, size_t control_port) {
    if (port == 0 || port > 65535 || control_port > 65535 || control_port == port)
        throw std::invalid_argument("Invalid RayFEM server/control port");
    const auto host = listen.find(':') != std::string::npos && !listen.starts_with('[')
        ? "[" + listen + "]" : listen;
    m_impl = std::make_unique<Impl>("tcp://" + host + ":" + std::to_string(port),
        control_port ? "tcp://" + host + ":" + std::to_string(control_port) : "", port);
}
Server::~Server() = default;
std::vector<std::shared_ptr<RenderFrame>> Server::fetchFrames(size_t max_frames) {
    std::vector<std::shared_ptr<RenderFrame>> result;
    if (max_frames == 0) return result;
    // Time budget is checked between complete frames: one unusually large frame
    // may take longer, but normal bursts cannot monopolize the UI indefinitely.
    const auto start = std::chrono::steady_clock::now();
    for (size_t attempt = 0; attempt < max_frames; ++attempt) {
        SerializedFrame serialized;
        {
            std::lock_guard lock(m_impl->mutex);
            if (m_impl->queue.empty()) break;
            serialized = std::move(m_impl->queue.front());
            m_impl->queued_bytes -= serialized.queued_bytes;
            m_impl->queue.pop_front();
        }
        try {
#ifndef MFEM_USE_EXCEPTIONS
            throw std::runtime_error("Streaming requires MFEM_USE_EXCEPTIONS for safe decoding");
#else
            if (mfem::get_error_action() != mfem::MFEM_ERROR_THROW)
                throw std::runtime_error("MFEM must use MFEM_ERROR_THROW for safe streaming");
            // Decoding constructs collections and spaces; see threading.hpp.
            std::lock_guard construction(FiniteElementConstructionMutex());
            auto mesh = serialized.mesh->decoded.lock();
            if (!mesh) {
                if (serialized.mesh->decode_failed) throw std::runtime_error("Mesh registration failed decoding");
                try {
                    validateMeshText(serialized.mesh->data);
                    std::istringstream stream(serialized.mesh->data);
                    // Preserve sender connectivity and DOF orientation: refinement
                    // marking or orientation repair can reorder simplex nodes.
                    mesh = std::make_shared<mfem::Mesh>(stream, 1, 0, false);
                    if (stream.fail() || mesh->GetNE() < 1 || mesh->Dimension() < 1 || mesh->Dimension() > 3 || mesh->SpaceDimension() > 3)
                        throw std::runtime_error("Invalid or unsupported mesh");
                    serialized.mesh->estimated_bytes = estimateMeshBytes(*mesh, serialized.mesh->data.size());
                    serialized.mesh->decoded = mesh;
                } catch (...) { serialized.mesh->decode_failed = true; throw; }
            }
            auto frame = std::make_shared<RenderFrame>();
            frame->metadata = serialized.header.metadata;
            frame->mesh_id = serialized.header.mesh_id;
            frame->session_id = serialized.header.session_id;
            frame->mesh_revision = serialized.header.mesh_revision;
            frame->mesh = std::move(mesh);
            frame->estimated_mesh_bytes = serialized.mesh->estimated_bytes;
            const bool binary = serialized.header.flags & Protocol::BinaryFields;
            for (const auto& [name, data] : serialized.fields) {
                auto decoded = decodeField(frame->mesh, data, binary, m_impl->spaces);
                // A shared space is counted once, with the frame that decoded it.
                frame->estimated_bytes += decoded.shared_space ? decoded.field->Size()*sizeof(mfem::real_t) :
                    estimateFieldBytes(*decoded.field, data.size());
                frame->fields.push_back({name, std::move(decoded.field)});
            }
            if (!serialized.displacement.empty()) {
                auto decoded = decodeField(frame->mesh, serialized.displacement, binary, m_impl->spaces);
                frame->displacement = std::move(decoded.field);
                auto* space = frame->displacement->FESpace();
                if (space->GetVDim() != frame->mesh->SpaceDimension() ||
                    space->FEColl()->GetRangeType(frame->mesh->Dimension()) != mfem::FiniteElement::SCALAR)
                    throw std::runtime_error("Displacement must be a pointwise vector field");
                frame->estimated_bytes += decoded.shared_space ? frame->displacement->Size()*sizeof(mfem::real_t) :
                    estimateFieldBytes(*frame->displacement, serialized.displacement.size());
            }
            ++m_impl->received;
            result.push_back(std::move(frame));
#endif
        } catch (const std::exception& exception) { m_impl->error(std::string("Frame rejected: ") + exception.what()); }
        if (std::chrono::steady_clock::now() - start > std::chrono::milliseconds(12)) break;
    }
    return result;
}
std::shared_ptr<RenderFrame> Server::fetchLatestFrame() {
    {
        std::lock_guard lock(m_impl->mutex);
        while (m_impl->queue.size() > 1) {
            m_impl->queued_bytes -= m_impl->queue.front().queued_bytes;
            m_impl->queue.pop_front();
            ++m_impl->dropped;
        }
    }
    auto frames = fetchFrames(1);
    return frames.empty() ? nullptr : std::move(frames.back());
}
size_t Server::getReceivedFrameCount() const { return m_impl->received; }
size_t Server::getDroppedFrameCount() const { return m_impl->dropped; }
size_t Server::getErrorCount() const { return m_impl->errors; }
size_t Server::getPendingFrameCount() const { std::lock_guard lock(m_impl->mutex); return m_impl->queue.size(); }
void Server::setReceiving(bool receiving) {
    m_impl->receiving = receiving;
    if (receiving) return;
    std::lock_guard lock(m_impl->mutex);
    m_impl->skipped += m_impl->queue.size();
    m_impl->queue.clear();
    m_impl->queued_bytes = 0;
}
bool Server::receiving() const { return m_impl->receiving; }
size_t Server::getSkippedFrameCount() const { return m_impl->skipped; }
bool Server::ready() const { return m_impl->is_ready; }
void Server::markViewerReady(bool ready) { m_impl->viewer_ready = ready; }
bool Server::shutdownRequested() const { return m_impl->shutdown_requested; }
std::string Server::getLastError() const { std::lock_guard lock(m_impl->mutex); return m_impl->last_error; }
} // namespace RayFEM
