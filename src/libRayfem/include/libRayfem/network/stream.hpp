#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
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

// Streams meshes and frames without MFEM: meshes are MFEM mesh text and
// fields are FieldData. This is what language bindings and other producers
// use; Client (client.hpp) serializes MFEM objects and sends through it.
// The viewer validates everything it decodes; this checks what it can
// without MFEM. One StreamClient must be used from one thread.
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
    bool fail(std::string message);

private:
    struct Registration { uint64_t revision = 0; uint32_t last_frame = 0; bool has_frame = false; };
    bool sendPacket(Protocol::Header header, const std::string& payload);
    bool sendSchema();
    struct Socket;
    std::unique_ptr<Socket> m_socket;
    uint64_t m_sessionId = 0, m_nextRevision = 0, m_schemaRevision = 0;
    std::unordered_map<uint64_t, Registration> m_meshes;
    std::optional<FrameSchema> m_schema;
    std::string m_lastError;
    bool m_binaryFields = true;
};
} // namespace RayFEM
