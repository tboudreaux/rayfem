#include "libRayfem/network/client.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace RayFEM {
namespace {
std::string spaceHeader(const mfem::FiniteElementSpace& space) {
    std::ostringstream header;
    header << std::setprecision(17);
    space.Save(header);
    return header.str();
}
}
Client::~Client() = default;
Client::Client(Client&&) noexcept = default;
Client& Client::operator=(Client&&) noexcept = default;
Client::Client(const std::string& host, size_t port) : m_stream(std::make_unique<StreamClient>(host, port)) {}
bool Client::fail(std::string message) { return m_stream->fail(std::move(message)); }
const std::string& Client::getLastError() const { return m_stream->getLastError(); }
void Client::setBinaryFields(bool binary) { m_stream->setBinaryFields(binary); }
bool Client::registerSchema(const FrameSchema& schema) { return m_stream->registerSchema(schema); }
const std::optional<FrameSchema>& Client::schema() const { return m_stream->schema(); }
bool Client::sendMesh(uint64_t mesh_id, mfem::Mesh& mesh) {
    if (mesh.GetNE() == 0 || mesh.Dimension() < 1 || mesh.Dimension() > 3 || mesh.SpaceDimension() > 3)
        return fail("Only nonempty meshes in one to three dimensions are supported");
    std::ostringstream stream;
    stream << std::setprecision(17);
    mesh.Print(stream);
    if (!m_stream->sendMesh(mesh_id, stream.str())) return false;
    m_meshes[mesh_id] = {&mesh, mesh.GetSequence(), mesh.GetNV(), mesh.GetNE()};
    return true;
}
bool Client::sendFrame(uint64_t mesh_id, const std::vector<NamedField>& fields,
                       const FrameMetadata& metadata, mfem::GridFunction* displacement) {
    auto found = m_meshes.find(mesh_id);
    if (found == m_meshes.end()) return fail("Call sendMesh before sendFrame");
    const Registration& registration = found->second;
    auto* mesh = registration.mesh;
    if (mesh->GetSequence() != registration.sequence || mesh->GetNV() != registration.vertices || mesh->GetNE() != registration.elements)
        return fail("Mesh topology changed; register the mesh again");
    auto valid = [&](mfem::GridFunction* gf) {
        return gf && gf->FESpace() && gf->FESpace()->GetMesh() == mesh && gf->Size() == gf->FESpace()->GetVSize();
    };
    // Headers are kept here so the FieldData views stay valid while sending.
    std::vector<std::string> headers;
    headers.reserve(fields.size() + 1);
    std::vector<FieldData> data;
    data.reserve(fields.size());
    for (const auto& field : fields) {
        if (!valid(field.gf)) return fail("Field is invalid, non-finite, or belongs to a different mesh");
        headers.push_back(spaceHeader(*field.gf->FESpace()));
        data.push_back({field.name, headers.back(), {field.gf->HostRead(), static_cast<size_t>(field.gf->Size())}});
    }
    FieldData moved;
    if (displacement) {
        if (!valid(displacement) || displacement->FESpace()->GetVDim() != mesh->SpaceDimension() ||
            displacement->FESpace()->FEColl()->GetRangeType(mesh->Dimension()) != mfem::FiniteElement::SCALAR)
            return fail("Displacement must be a pointwise vector field on the registered mesh");
        headers.push_back(spaceHeader(*displacement->FESpace()));
        moved = {"displacement", headers.back(), {displacement->HostRead(), static_cast<size_t>(displacement->Size())}};
    }
    return m_stream->sendFrame(mesh_id, data, metadata, displacement ? &moved : nullptr);
}
bool Client::send(mfem::Mesh& mesh, mfem::GridFunction& gf, const FrameMetadata& metadata) {
    return sendMesh(0, mesh) && sendFrame(0, {{"solution", &gf}}, metadata);
}
} // namespace RayFEM
