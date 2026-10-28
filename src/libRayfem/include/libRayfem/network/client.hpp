#pragma once

#include <string>
#include <memory>
#include <unordered_map>
#include <vector>
#include "mfem.hpp"
#include "libRayfem/network/packet.hpp"

namespace RayFEM {
struct NamedField {
    std::string name;
    mfem::GridFunction* gf = nullptr;
};

// A Client and its MFEM objects must be used from one thread.
class Client {
public:
    Client(const std::string& host, size_t port);
    ~Client();
    Client(Client&&) noexcept;
    Client& operator=(Client&&) noexcept;
    Client(const Client&) = delete;
    Client& operator=(const Client&) = delete;
    bool sendMesh(uint64_t mesh_id, mfem::Mesh& mesh);
    bool sendFrame(uint64_t mesh_id, const std::vector<NamedField>& fields,
                   const FrameMetadata& metadata, mfem::GridFunction* displacement = nullptr);
    // Binary fields (the default) decode far faster and are about a third the
    // size of MFEM text. Viewers older than the binary encoding need text.
    void setBinaryFields(bool binary) { m_binaryFields = binary; }
    // Compatibility helper: registers fresh geometry and sends a single field.
    bool send(mfem::Mesh& mesh, mfem::GridFunction& gf, const FrameMetadata& metadata);
    const std::string& getLastError() const { return m_lastError; }
private:
    struct Registration {
        mfem::Mesh* mesh = nullptr;
        uint64_t revision = 0;
        long sequence = 0;
        int vertices = 0;
        int elements = 0;
        uint32_t last_frame = 0;
        bool has_frame = false;
    };
    bool sendPacket(Protocol::Header header, const std::string& payload);
    bool fail(std::string message);
    struct Socket;
    std::unique_ptr<Socket> m_socket;
    uint64_t m_sessionId = 0;
    uint64_t m_nextRevision = 0;
    std::unordered_map<uint64_t, Registration> m_meshes;
    std::string m_lastError;
    bool m_binaryFields = true;
};
} // namespace RayFEM
