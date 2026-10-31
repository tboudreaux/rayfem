#pragma once

#include <string>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>
#include "mfem.hpp"
#include "libRayfem/network/stream.hpp"

namespace RayFEM {
struct NamedField {
    std::string name;
    mfem::GridFunction* gf = nullptr;
};

// Streams MFEM objects: it serializes meshes and grid functions and sends them
// through a StreamClient (stream.hpp), checking first that they are
// consistent with the registered mesh. A Client and its MFEM objects must be
// used from one thread.
class Client {
public:
    Client(const std::string& host, size_t port);
    ~Client();
    Client(Client&&) noexcept;
    Client& operator=(Client&&) noexcept;
    Client(const Client&) = delete;
    Client& operator=(const Client&) = delete;
    // Declare the values and tags frames will carry (FrameMetadata::values
    // and tags). Register before sending frames that use them; registering
    // again replaces the schema. It is re-sent with each mesh registration
    // so a viewer started or restarted later still receives it.
    bool registerSchema(const FrameSchema& schema);
    const std::optional<FrameSchema>& schema() const;
    bool sendMesh(uint64_t mesh_id, mfem::Mesh& mesh);
    bool sendFrame(uint64_t mesh_id, const std::vector<NamedField>& fields,
                   const FrameMetadata& metadata, mfem::GridFunction* displacement = nullptr);
    // Binary fields (the default) decode far faster and are about a third the
    // size of MFEM text. Viewers older than the binary encoding need text.
    void setBinaryFields(bool binary);
    // Compatibility helper: registers fresh geometry and sends a single field.
    bool send(mfem::Mesh& mesh, mfem::GridFunction& gf, const FrameMetadata& metadata);
    const std::string& getLastError() const;
    // Whether a viewer is connected now (see StreamClient).
    bool connected() { return m_stream->connected(); }
    // Sending in the background; see StreamClient::startQueue. A frame is
    // encoded before sendFrame returns, so the fields may change right after.
    void startQueue(const StreamQueue& options = {}) { m_stream->startQueue(options); }
    bool flush(int timeout_ms = -1) { return m_stream->flush(timeout_ms); }
    StreamQueueStats queueStats() const { return m_stream->queueStats(); }
    // The MFEM-free client underneath.
    StreamClient& stream() { return *m_stream; }
private:
    struct Registration {
        mfem::Mesh* mesh = nullptr;
        long sequence = 0;
        int vertices = 0;
        int elements = 0;
    };
    bool fail(std::string message);
    std::unique_ptr<StreamClient> m_stream;
    std::unordered_map<uint64_t, Registration> m_meshes;
};
} // namespace RayFEM
