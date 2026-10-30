#include "libRayfem/frame/tracking.hpp"  // before zmq.hpp: its syslog macros clash with raylib
#include "libRayfem/network/client.hpp"
#include "libRayfem/network/server.hpp"
#include "libRayfem/network/zmq.hpp"
#include <bit>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unistd.h>

namespace {
void require(bool value, const std::string& message) {
    if (!value) throw std::runtime_error(message);
}
void waitFor(const std::function<bool()>& condition, const std::string& message) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    while (!condition()) {
        if (std::chrono::steady_clock::now() > deadline) throw std::runtime_error(message);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
std::vector<std::shared_ptr<RayFEM::RenderFrame>> receive(RayFEM::Server& server, size_t count) {
    std::vector<std::shared_ptr<RayFEM::RenderFrame>> result;
    waitFor([&] {
        for (auto& frame : server.fetchFrames(8)) result.push_back(std::move(frame));
        return result.size() >= count;
    }, "Timed out receiving frames: " + server.getLastError());
    return result;
}
void sendRaw(RayFEM::SocketPtr& socket, RayFEM::Protocol::Header header, const std::string& payload) {
    header.payload_bytes = payload.size();
    std::string bytes = RayFEM::Protocol::encodeHeader(header) + payload;
    RayFEM::FramePtr packet(zframe_new(bytes.data(), bytes.size()));
    auto* raw = packet.release();
    const int status = zframe_send(&raw, socket, 0);
    if (raw) zframe_destroy(&raw);
    require(status == 0, "Could not send test packet");
}
std::string binaryField(const std::string& header, uint64_t count, const std::vector<double>& values,
                        std::string_view magic = RayFEM::Protocol::BinaryFieldMagic) {
    std::string out(magic);
    RayFEM::Protocol::appendInteger(out, header.size(), 4);
    out += header;
    RayFEM::Protocol::appendInteger(out, count, 8);
    for (double value : values) RayFEM::Protocol::appendInteger(out, std::bit_cast<uint64_t>(value), 8);
    return out;
}
std::string fieldPayload(const std::string& name, const std::string& data) {
    std::string payload;
    RayFEM::Protocol::appendInteger(payload, name.size(), 4);
    RayFEM::Protocol::appendInteger(payload, data.size(), 8);
    return payload + name + data;
}
RayFEM::FrameSchema iterationSchema() {
    RayFEM::FrameSchema schema;
    schema.values = {{"iteration", RayFEM::ValueType::Integer, true, "", "Newton iteration"},
                     {"virial ratio", RayFEM::ValueType::Real, false, "", "2K/|W|"},
                     {"radius", RayFEM::ValueType::Real, true, "R_sun", ""},
                     {"solver", RayFEM::ValueType::Text, false, "", ""}};
    schema.tags = {{"iterate", "Intermediate"}, {"converged", "Converged state"}};
    schema.has_time = false;
    schema.primary = "iteration";
    return schema;
}
void testSchemaEncoding() {
    using namespace RayFEM;
    const auto schema = iterationSchema();
    std::string error;
    require(Protocol::validateSchema(schema, error), error);
    FrameSchema decoded;
    require(Protocol::decodeSchema(Protocol::encodeSchema(schema), decoded, error), error);
    require(decoded.values.size() == 4 && decoded.tags.size() == 2 && !decoded.has_time && decoded.primary == "iteration" &&
            decoded.values[2].unit == "R_sun" && decoded.values[0].coordinate && decoded.values[3].type == ValueType::Text &&
            decoded.tags[1].description == "Converged state", "Schema round trip");
    auto broken = schema;
    broken.values.push_back({"label", ValueType::Text, true});
    require(!Protocol::validateSchema(broken, error), "Text coordinates are rejected");
    broken = schema; broken.values.push_back(broken.values.front());
    require(!Protocol::validateSchema(broken, error), "Duplicate value names are rejected");
    broken = schema; broken.primary = "virial ratio";
    require(!Protocol::validateSchema(broken, error), "A primary that is not a coordinate is rejected");
    std::string truncated = Protocol::encodeSchema(schema);
    truncated.pop_back();
    require(!Protocol::decodeSchema(truncated, decoded, error), "Truncated schemas are rejected");
    // Descriptors: integers widen to reals; types, names and tags are checked.
    FrameMetadata metadata;
    metadata.values = {{"virial ratio", int64_t{1}}, {"iteration", int64_t{4}}, {"solver", std::string("Newton")}};
    metadata.tags = {"converged"};
    std::string bytes;
    require(Protocol::encodeDescriptor(schema, metadata, bytes, error), error);
    FrameMetadata back;
    require(Protocol::decodeDescriptor(schema, bytes, back, error), error);
    require(back.number("virial ratio") == 1.0 && std::holds_alternative<double>(*back.find("virial ratio")) &&
            std::get<int64_t>(*back.find("iteration")) == 4 && std::get<std::string>(*back.find("solver")) == "Newton" &&
            back.hasTag("converged") && !back.hasTag("iterate") && !back.find("radius"), "Descriptor round trip");
    for (const auto& bad : {FrameMetadata{0, 0, "", {{"unknown", 1.0}}, {}}, FrameMetadata{0, 0, "", {{"iteration", 1.5}}, {}},
                            FrameMetadata{0, 0, "", {{"solver", 2.0}}, {}}, FrameMetadata{0, 0, "", {{"radius", std::nan("")}}, {}},
                            FrameMetadata{0, 0, "", {}, {"diverged"}}, FrameMetadata{0, 0, "", {}, {"iterate", "iterate"}}}) {
        std::string ignored;
        require(!Protocol::encodeDescriptor(schema, bad, ignored, error), "Invalid frame values or tags are rejected");
    }
    bytes.push_back('x');
    require(!Protocol::decodeDescriptor(schema, bytes, back, error), "Trailing descriptor bytes are rejected");
}
void testWireHeader() {
    RayFEM::Protocol::Header expected;
    expected.session_id = 0x1122334455667788ULL;
    expected.mesh_id = 0x8877665544332211ULL;
    expected.mesh_revision = 3;
    expected.field_count = 1;
    expected.metadata.frame_index = 42;
    expected.metadata.sim_time = 0.125;
    expected.payload_bytes = 3;
    std::string bytes = RayFEM::Protocol::encodeHeader(expected) + "abc";
    require(bytes.size() == 131, "Unexpected wire header size");
    require(static_cast<unsigned char>(bytes[24]) == 0x11 && static_cast<unsigned char>(bytes[31]) == 0x88,
            "Session ID is not network byte order");
    RayFEM::Protocol::Header actual;
    std::string error;
    require(RayFEM::Protocol::decodeHeader(bytes, actual, error), error);
    require(actual.session_id == expected.session_id && actual.mesh_id == expected.mesh_id && actual.metadata.sim_time == 0.125,
            "Header round trip failed");
    require(!RayFEM::Protocol::decodeHeader(bytes.substr(0, 127), actual, error), "Truncated header accepted");
    bytes[9] = 99;
    require(!RayFEM::Protocol::decodeHeader(bytes, actual, error), "Future protocol version accepted");
}
}

int main(int argc, char** argv) {
    try {
        testWireHeader();
        testSchemaEncoding();
        const size_t port = 20000 + (getpid() % 20000);
        RayFEM::Server server("127.0.0.1", port);
        waitFor([&] { return server.ready() || server.getErrorCount() > 0; }, "Server startup timed out");
        require(server.ready(), server.getLastError());
        RayFEM::Client client("127.0.0.1", port);
        mfem::Mesh mesh = mfem::Mesh::MakeCartesian2D(3, 3, mfem::Element::QUADRILATERAL, true, 1.0, 1.0);
        mesh.SetCurvature(4);
        mesh.Transform([](const mfem::Vector& x, mfem::Vector& y) {
            y.SetSize(2);
            y[0] = x[0] + 0.08 * std::sin(3.141592653589793 * x[1]);
            y[1] = x[1] + 0.05 * x[0] * x[0];
        });
        mfem::H1_FECollection h1(3, 2);
        mfem::L2_FECollection l2(2, 2);
        mfem::ND_FECollection nd(2, 2);
        mfem::RT_FECollection rt(1, 2);
        mfem::FiniteElementSpace scalar_space(&mesh, &h1), l2_space(&mesh, &l2), nd_space(&mesh, &nd), rt_space(&mesh, &rt);
        mfem::FiniteElementSpace vector_space(&mesh, &h1, 2, mfem::Ordering::byVDIM);
        mfem::GridFunction scalar(&scalar_space), discontinuous(&l2_space), curl_field(&nd_space), div_field(&rt_space), vector(&vector_space), displacement(&vector_space);
        mfem::FunctionCoefficient scalar_coefficient([](const mfem::Vector& x) { return x[0] + 2.0 * x[1]; });
        mfem::VectorFunctionCoefficient vector_coefficient(2, [](const mfem::Vector& x, mfem::Vector& v) { v.SetSize(2); v[0] = -x[1]; v[1] = x[0]; });
        scalar.ProjectCoefficient(scalar_coefficient);
        discontinuous.ProjectCoefficient(scalar_coefficient);
        curl_field.ProjectCoefficient(vector_coefficient);
        div_field.ProjectCoefficient(vector_coefficient);
        vector.ProjectCoefficient(vector_coefficient);
        displacement = 0.025;
        const std::vector<RayFEM::NamedField> fields{{"temperature", &scalar}, {"DG", &discontinuous}, {"Hcurl", &curl_field}, {"Hdiv", &div_field}, {"velocity", &vector}};
        RayFEM::FrameMetadata metadata;
        require(!client.sendFrame(7, fields, metadata), "Unregistered mesh accepted");
        require(client.sendMesh(7, mesh), client.getLastError());
        require(!client.sendFrame(7, {{"duplicate", &scalar}, {"duplicate", &scalar}}, metadata), "Duplicate fields accepted");
        require(!client.sendFrame(7, {{"null", nullptr}}, metadata), "Null field accepted");
        require(!client.sendFrame(7, fields, metadata, &curl_field), "Hcurl displacement accepted");
        for (uint32_t index = 0; index < 3; ++index) {
            metadata.frame_index = index;
            metadata.sim_time = index * 0.1;
            require(client.sendFrame(7, fields, metadata, &displacement), client.getLastError());
        }
        require(!client.sendFrame(7, fields, metadata), "Duplicate frame index accepted");
        auto first = receive(server, 3);
        for (size_t i = 0; i < first.size(); ++i) {
            require(first[i]->metadata.frame_index == i && first[i]->fields.size() == fields.size(), "Atomic frame metadata or fields changed");
            require(first[i]->mesh == first.front()->mesh, "Cached mesh was decoded more than once");
            require(first[i]->mesh->GetNodes() && first[i]->mesh->GetNodes()->FESpace()->GetMaxElementOrder() == 4, "Curvilinear geometry lost its order");
            for (size_t j = 0; j < fields.size(); ++j) {
                require(first[i]->fields[j].name == fields[j].name, "Field name changed");
                auto& decoded = *first[i]->fields[j].gf;
                require(decoded.Size() == fields[j].gf->Size(), "Field size changed");
                for (int k = 0; k < decoded.Size(); ++k)
                    require(std::abs(decoded[k] - (*fields[j].gf)[k]) < 1e-12, "Field coefficient changed");
            }
            require(first[i]->displacement && std::abs((*first[i]->displacement)[0] - 0.025) < 1e-12, "Displacement was lost");
        }
        size_t shared_history_bytes = first.front()->estimated_mesh_bytes;
        require(shared_history_bytes > 0, "Decoded geometry must have a retention estimate");
        for (const auto& frame : first) {
            require(frame->estimated_bytes > 0, "Decoded fields must have a retention estimate");
            shared_history_bytes += frame->estimated_bytes;
        }
        require(RayFEM::EstimateHistoryBytes(first) == shared_history_bytes, "Shared mesh must be counted once in history estimate");
        // Binary frames of one registration share their decoded spaces, and
        // later frames count only their coefficients.
        for (size_t j = 0; j < fields.size(); ++j)
            require(first[2]->fields[j].gf->FESpace() == first[0]->fields[j].gf->FESpace(), "Frames did not share a decoded space");
        require(first[2]->estimated_bytes < first[0]->estimated_bytes, "Shared spaces were counted for every frame");
        // MFEM text remains supported, decodes to the same values, and shares spaces too.
        client.setBinaryFields(false);
        require(client.sendMesh(8, mesh), client.getLastError());
        for (uint32_t index = 0; index < 2; ++index) {
            metadata.frame_index = index;
            require(client.sendFrame(8, fields, metadata, &displacement), client.getLastError());
        }
        client.setBinaryFields(true);
        auto text = receive(server, 2);
        for (const auto& frame : text)
            for (size_t j = 0; j < fields.size(); ++j)
                for (int k = 0; k < frame->fields[j].gf->Size(); ++k)
                    require((*frame->fields[j].gf)[k] == (*first[0]->fields[j].gf)[k], "Text and binary fields decoded differently");
        require(text[1]->fields[0].gf->FESpace() == text[0]->fields[0].gf->FESpace(), "Text frames did not share a decoded space");
        metadata.frame_index = 2;
        for (uint32_t index = 3; index < 123; ++index) {
            metadata.frame_index = index;
            scalar = double(index);
            require(client.sendFrame(7, fields, metadata, &displacement), client.getLastError());
        }
        waitFor([&] { return server.getDroppedFrameCount() >= 56; }, "Burst did not exercise bounded queue");
        require(server.getPendingFrameCount() <= 64, "Receiver queue exceeded bound");
        auto burst = receive(server, 64);
        require(burst.front()->metadata.frame_index == 59 && burst.back()->metadata.frame_index == 122, "Newest complete frames were not retained in order");
        for (size_t i = 0; i < burst.size(); ++i) {
            require(burst[i]->metadata.frame_index == 59 + i, "Burst frame ordering changed");
            require((*burst[i]->fields[0].gf)[0] == double(59 + i), "Fields crossed frame boundaries");
        }
        RayFEM::SocketPtr raw(zsock_new(ZMQ_PUSH));
        zsock_set_linger(raw, 0);
        zsock_set_sndtimeo(raw, 1000);
        const std::string address = "tcp://127.0.0.1:" + std::to_string(port);
        require(zsock_connect(raw, "%s", address.c_str()) == 0, "Could not connect malformed-packet sender");
        RayFEM::Protocol::Header header;
        header.session_id = 777;
        header.mesh_id = 55;
        header.mesh_revision = 1;
        header.field_count = 1;
        size_t errors = server.getErrorCount();
        sendRaw(raw, header, fieldPayload("missing mesh", "invalid"));
        waitFor([&] { return server.getErrorCount() > errors; }, "Missing mesh reference was not rejected");
        header.kind = RayFEM::Protocol::Kind::Mesh;
        header.field_count = 0;
        std::ostringstream mesh_data;
        mesh.Print(mesh_data);
        sendRaw(raw, header, mesh_data.str());
        header.kind = RayFEM::Protocol::Kind::Frame;
        header.field_count = 1;
        errors = server.getErrorCount();
        sendRaw(raw, header, fieldPayload("truncated", "FiniteElementSpace\nFiniteElementCollection: H1_2D_P3\nVDim: 1\nOrdering: 0\n0\n"));
        waitFor([&] { server.fetchFrames(); return server.getErrorCount() > errors; }, "Truncated MFEM field was not rejected");
        errors = server.getErrorCount();
        header.metadata.frame_index = 1;
        header.mesh_revision = 99;
        sendRaw(raw, header, fieldPayload("wrong revision", "invalid"));
        waitFor([&] { return server.getErrorCount() > errors; }, "Wrong mesh revision was not rejected");
        header.mesh_revision = 1;
        header.metadata.frame_index = 2;
        errors = server.getErrorCount();
        sendRaw(raw, header, fieldPayload("invalid dimension", "FiniteElementSpace\nFiniteElementCollection: H1_2D_P3\nVDim: -1\nOrdering: 0\n0\n"));
        waitFor([&] { server.fetchFrames(); return server.getErrorCount() > errors; }, "Invalid field dimension was not rejected");
        header.metadata.frame_index = 3;
        errors = server.getErrorCount();
        sendRaw(raw, header, fieldPayload("trailing bytes", "invalid") + "unexpected");
        waitFor([&] { return server.getErrorCount() > errors; }, "Trailing packet bytes were not rejected");
        {
            const std::string space = "FiniteElementSpace\nFiniteElementCollection: H1_2D_P3\nVDim: 1\nOrdering: 0\n";
            const std::vector<double> ones(100, 1.0), short_values(99, 1.0);
            std::vector<double> invalid(100, 1.0);
            invalid[50] = std::nan("");
            header.flags = RayFEM::Protocol::BinaryFields;
            header.metadata.frame_index = 4;
            sendRaw(raw, header, fieldPayload("binary", binaryField(space, 100, ones)));
            auto binary = receive(server, 1);
            require(binary.front()->fields[0].gf->Size() == 100 && (*binary.front()->fields[0].gf)[99] == 1.0, "Valid binary field was not decoded");
            const std::pair<std::string, std::string> rejected[] = {
                {"bad magic", binaryField(space, 100, ones, "XXXX")},
                {"truncated values", binaryField(space, 100, short_values)},
                {"size mismatch", binaryField(space, 99, short_values)},
                {"non-finite", binaryField(space, 100, invalid)},
                {"bad space", binaryField("FiniteElementSpace\nFiniteElementCollection: H1_2D_P3\nVDim: -1\nOrdering: 0\n", 100, ones)},
            };
            for (const auto& [name, data] : rejected) {
                ++header.metadata.frame_index;
                errors = server.getErrorCount();
                sendRaw(raw, header, fieldPayload(name, data));
                waitFor([&] { server.fetchFrames(); return server.getErrorCount() > errors; }, "Malformed binary field was accepted: " + name);
            }
            header.flags = 0;
        }
        auto rejectMesh = [&](uint64_t id, const std::string& data) {
            header.kind = RayFEM::Protocol::Kind::Mesh;
            header.mesh_id = id;
            header.field_count = 0;
            header.metadata.frame_index = 0;
            sendRaw(raw, header, data);
            header.kind = RayFEM::Protocol::Kind::Frame;
            header.field_count = 1;
            const size_t previous_errors = server.getErrorCount();
            sendRaw(raw, header, fieldPayload("bad mesh", "invalid"));
            waitFor([&] { server.fetchFrames(); return server.getErrorCount() > previous_errors; }, "Malformed mesh was not rejected");
        };
        rejectMesh(56, "MFEM mesh v1.0\ndimension\n4\nelements\n1\n1 3 0 1 2 3\nboundary\n0\nvertices\n4\n4\n");
        rejectMesh(57, "MFEM mesh v1.0\ndimension\n2\nelements\n1\n1 2 0 1 999\nboundary\n0\nvertices\n3\n2\n0 0 1 0 0 1\n");
        rejectMesh(58, "MFEM NURBS mesh v1.0\ndimension\n2\nelements\n-3\n");
        metadata.frame_index = 123;
        require(client.sendFrame(7, fields, metadata), client.getLastError());
        auto recovered = receive(server, 1);
        require(recovered.front()->metadata.frame_index == 123, "Receiver did not recover after malformed input");
        require(!recovered.front()->displacement, "Previous frame displacement leaked into next frame");
        // Re-registering geometry explicitly starts a fresh frame sequence.
        require(client.sendMesh(7, mesh), client.getLastError());
        metadata.frame_index = 0;
        require(client.sendFrame(7, fields, metadata), client.getLastError());
        auto reset = receive(server, 1);
        require(reset.front()->mesh_revision > first.front()->mesh_revision && reset.front()->mesh != first.front()->mesh,
                "Mesh registration revision was not respected");
        const std::vector<std::shared_ptr<RayFEM::RenderFrame>> distinct_history{first.front(), reset.front()};
        require(RayFEM::EstimateHistoryBytes(distinct_history) == first.front()->estimated_bytes + reset.front()->estimated_bytes +
                first.front()->estimated_mesh_bytes + reset.front()->estimated_mesh_bytes,
                "Distinct mesh registrations must both count toward retained memory");
        // The receiver must not run MFEM refinement marking, which can
        // permute simplex local vertices independently of the streamed field.
        mfem::Mesh triangle(2, 3, 1, 3);
        triangle.AddVertex(0.0, 0.0);
        triangle.AddVertex(3.0, 0.0);
        triangle.AddVertex(0.0, 1.0);
        triangle.AddTriangle(0, 1, 2);
        triangle.AddBdrSegment(0, 1);
        triangle.AddBdrSegment(1, 2);
        triangle.AddBdrSegment(2, 0);
        triangle.FinalizeTriMesh(1, 0, false);
        triangle.SetCurvature(3);
        mfem::FiniteElementSpace triangle_space(&triangle, &h1);
        mfem::GridFunction triangle_field(&triangle_space);
        triangle_field.ProjectCoefficient(scalar_coefficient);
        require(client.sendMesh(9, triangle), client.getLastError());
        require(client.sendFrame(9, {{"simplex", &triangle_field}}, metadata), client.getLastError());
        auto simplex = receive(server, 1);
        mfem::Array<int> sender_vertices, receiver_vertices;
        triangle.GetElementVertices(0, sender_vertices);
        simplex[0]->mesh->GetElementVertices(0, receiver_vertices);
        for (int i = 0; i < sender_vertices.Size(); ++i)
            require(sender_vertices[i] == receiver_vertices[i], "Simplex vertex order changed during decode");
        const auto& center = mfem::Geometries.GetCenter(mfem::Geometry::TRIANGLE);
        require(std::abs(simplex[0]->fields[0].gf->GetValue(0, center) - triangle_field.GetValue(0, center)) < 1e-12,
                "Simplex field orientation changed during decode");
        if (argc > 1) {
            std::ifstream input(argv[1]);
            require(input.good(), "NURBS mesh fixture was not found");
            mfem::Mesh nurbs(input, 1, 1);
            require(nurbs.NURBSext && nurbs.GetNodes(), "Fixture is not NURBS");
            mfem::GridFunction nurbs_field(nurbs.GetNodes()->FESpace());
            nurbs_field = *nurbs.GetNodes();
            require(client.sendMesh(8, nurbs), client.getLastError());
            require(client.sendFrame(8, {{"NURBS coordinates", &nurbs_field}}, metadata), client.getLastError());
            auto nurbs_frame = receive(server, 1);
            require(nurbs_frame.front()->mesh->NURBSext != nullptr, "NURBS geometry was lost");
            require(nurbs_frame.front()->fields.front().gf->Size() == nurbs_field.Size(), "NURBS coefficients changed size");
            std::ostringstream saved_nurbs;
            nurbs.Print(saved_nurbs);
            const std::string valid_nurbs = saved_nurbs.str();
            const size_t knots = valid_nurbs.find("\nknotvectors\n");
            require(knots != std::string::npos, "NURBS fixture must include explicit knot vectors");
            const size_t count_start = knots + std::strlen("\nknotvectors\n");
            const size_t first_vector = valid_nurbs.find('\n', count_start) + 1;
            const size_t degree_end = valid_nurbs.find(' ', first_vector);
            const size_t control_end = valid_nurbs.find(' ', degree_end + 1);
            std::string excessive_degree = valid_nurbs;
            excessive_degree.replace(first_vector, degree_end - first_vector, "11");
            rejectMesh(59, excessive_degree);
            std::string excessive_control_points = valid_nurbs;
            excessive_control_points.replace(degree_end + 1, control_end - degree_end - 1, "2147483647");
            rejectMesh(60, excessive_control_points);
            std::string negative_knot_count = valid_nurbs;
            negative_knot_count.replace(count_start, first_vector - count_start - 1, "-1");
            rejectMesh(61, negative_knot_count);
            header.kind = RayFEM::Protocol::Kind::Mesh;
            header.mesh_id = 62;
            header.field_count = 0;
            sendRaw(raw, header, valid_nurbs);
            header.kind = RayFEM::Protocol::Kind::Frame;
            header.field_count = 1;
            header.metadata.frame_index = 0;
            errors = server.getErrorCount();
            sendRaw(raw, header, fieldPayload("excessive NURBS degree", "FiniteElementSpace\nFiniteElementCollection: NURBS11\nVDim: 1\nOrdering: 0\n0\n"));
            waitFor([&] { server.fetchFrames(); return server.getErrorCount() > errors; }, "Excessive NURBS field degree was not rejected");
            header.metadata.frame_index = 1;
            errors = server.getErrorCount();
            sendRaw(raw, header, fieldPayload("excessive variable NURBS degree", "MFEM FiniteElementSpace v1.0\nFiniteElementCollection: NURBS\nVDim: 1\nOrdering: 0\nNURBS_orders\n2147483647 2\nEnd: MFEM FiniteElementSpace v1.0\n0\n"));
            waitFor([&] { server.fetchFrames(); return server.getErrorCount() > errors; }, "Excessive variable NURBS order was not rejected");
        }
        {
            auto large_mesh = mfem::Mesh::MakeCartesian2D(96, 96, mfem::Element::QUADRILATERAL, true);
            mfem::H1_FECollection large_collection(4, 2);
            mfem::FiniteElementSpace large_space(&large_mesh, &large_collection);
            mfem::FiniteElementSpace large_vector_space(&large_mesh, &large_collection, 2);
            mfem::GridFunction large_field(&large_space), large_displacement(&large_vector_space);
            const double sent_value = 1.234567891234567;
            large_field = sent_value;
            large_displacement = .01234567891234567;
            require(large_field.Size() > 100000, "Large-frame fixture is too small");
            require(client.sendMesh(10, large_mesh), client.getLastError());
            metadata.frame_index = 0;
            require(client.sendFrame(10, {{"large scalar", &large_field}, {"second view", &large_field}}, metadata, &large_displacement), client.getLastError());
            // Serialization must snapshot the frame, independent of later writes.
            large_field = -4;
            auto large = receive(server, 1);
            require(large.front()->fields.size() == 2 && large.front()->displacement, "Large frame lost a field or displacement");
            for (const auto& field : large.front()->fields) {
                require(field.gf->Size() == large_space.GetVSize(), "Large frame changed coefficient count");
                for (int index : {0, field.gf->Size()/2, field.gf->Size()-1})
                    require(std::abs((*field.gf)[index] - sent_value) < 1e-15, "Large frame coefficients were truncated or aliased");
            }
            require(large.front()->displacement->Size() == large_vector_space.GetVSize(), "Large displacement changed coefficient count");
        }
        {
            // Paused receiving skips frames but keeps mesh registrations.
            server.fetchFrames(64);
            server.setReceiving(false);
            require(!server.receiving(), "Receiving did not pause");
            const size_t skipped = server.getSkippedFrameCount();
            require(client.sendMesh(12, mesh), client.getLastError());
            RayFEM::FrameMetadata paused;
            for (uint32_t index = 0; index < 3; ++index) {
                paused.frame_index = index;
                require(client.sendFrame(12, fields, paused), client.getLastError());
            }
            waitFor([&] { return server.getSkippedFrameCount() >= skipped + 3; }, "Paused frames were not counted as skipped");
            require(server.fetchFrames(64).empty() && server.getPendingFrameCount() == 0, "Frames were delivered while paused");
            server.setReceiving(true);
            paused.frame_index = 3;
            require(client.sendFrame(12, fields, paused), client.getLastError());
            auto resumed = receive(server, 1);
            require(resumed.front()->metadata.frame_index == 3 && resumed.front()->fields.size() == fields.size(),
                    "Frames after resuming did not decode with the mesh registered while paused");
        }
        {
            // Described frames: schema, values and tags arrive with the frame.
            server.fetchFrames(64);
            RayFEM::Client described("127.0.0.1", port);
            RayFEM::FrameMetadata early;
            early.values = {{"iteration", int64_t{1}}};
            require(described.sendMesh(30, mesh), described.getLastError());
            require(!described.sendFrame(30, fields, early), "Values without a schema are refused by the client");
            require(described.registerSchema(iterationSchema()), described.getLastError());
            require(described.sendMesh(30, mesh), described.getLastError());
            RayFEM::FrameMetadata wrong;
            wrong.values = {{"virial ratio", std::string("high")}};
            require(!described.sendFrame(30, fields, wrong), "A wrongly typed value is refused by the client");
            for (uint32_t index = 0; index < 3; ++index) {
                RayFEM::FrameMetadata metadata;
                metadata.frame_index = index;
                metadata.values = {{"iteration", int64_t(index + 1)}, {"virial ratio", 1.0 + 0.1/(index + 1)}, {"radius", 0.5 + index}};
                metadata.tags = {index == 2 ? "converged" : "iterate"};
                require(described.sendFrame(30, fields, metadata), described.getLastError());
            }
            auto received = receive(server, 3);
            const auto& last = received.back();
            require(last->schema && !last->schema->has_time && last->schema->primary == "iteration", "Frames carry their schema");
            require(last->metadata.number("iteration") == 3.0 && last->metadata.hasTag("converged") &&
                    std::abs(*last->metadata.number("virial ratio") - (1.0 + 0.1/3)) < 1e-15 && !last->metadata.find("solver"),
                    "Frame values and tags arrive decoded");
            require(received.front()->metadata.hasTag("iterate") && received.front()->fields.size() == fields.size(),
                    "Described frames keep their fields");
            // Tracking a frame value records the frame's coordinates too.
            RayFEM::TrackingRequest request;
            request.field_name = "virial ratio";
            request.kind = RayFEM::MeasurementKind::FrameValue;
            RayFEM::FrameTracker tracker(request);
            for (auto& frame : received) tracker.append(*frame);
            require(tracker.samples().size() == 3 && tracker.samples().back().valid &&
                    std::abs(tracker.samples().back().value - (1.0 + 0.1/3)) < 1e-15 &&
                    tracker.samples().back().coordinate("iteration") == 3.0 && tracker.samples().back().coordinate("radius") == 2.5 &&
                    !tracker.samples().back().coordinate("virial ratio"), "Frame values are tracked with their coordinates");
            // Registering again replaces the schema for later frames.
            auto revised = iterationSchema();
            revised.tags.push_back({"restart", ""});
            require(described.registerSchema(revised), described.getLastError());
            RayFEM::FrameMetadata restarted;
            restarted.frame_index = 3;
            restarted.tags = {"restart"};
            require(described.sendFrame(30, fields, restarted), described.getLastError());
            require(receive(server, 1).front()->metadata.hasTag("restart"), "A revised schema applies to later frames");

            // A descriptor from a session with no schema is rejected.
            RayFEM::SocketPtr raw(zsock_new(ZMQ_PUSH));
            zsock_set_linger(raw, 0);
            zsock_set_sndtimeo(raw, 1000);
            require(zsock_connect(raw, "tcp://127.0.0.1:%d", int(port)) == 0, "Could not connect descriptor sender");
            RayFEM::Protocol::Header header;
            header.session_id = 4242; header.mesh_id = 1; header.mesh_revision = 1;
            header.kind = RayFEM::Protocol::Kind::Mesh;
            std::ostringstream mesh_data; mesh_data << std::setprecision(17); mesh.Print(mesh_data);
            sendRaw(raw, header, mesh_data.str());
            std::ostringstream space; space << std::setprecision(17); scalar_space.Save(space);
            std::vector<double> values(scalar.Size(), 1.0);
            std::string descriptor;
            RayFEM::Protocol::appendInteger(descriptor, 0, 2);
            RayFEM::Protocol::appendInteger(descriptor, 0, 2);
            header.kind = RayFEM::Protocol::Kind::Frame; header.field_count = 1;
            header.flags = RayFEM::Protocol::BinaryFields | RayFEM::Protocol::HasDescriptor;
            const size_t errors = server.getErrorCount();
            sendRaw(raw, header, fieldPayload("u", binaryField(space.str(), values.size(), values)) + descriptor);
            waitFor([&] { return server.getErrorCount() > errors; }, "A descriptor without a schema was accepted");
            require(server.getLastError().find("schema") != std::string::npos, "The schema error is reported");
        }
        std::cout << "Streaming checks passed: curvilinear geometry, five FE spaces, shared meshes, atomic fields, displacement, burst queue, malformed input, restart, NURBS, and 148225-DOF multi-field frames.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Streaming test failed: " << error.what() << '\n';
        return 1;
    }
}
