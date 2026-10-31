// Python bindings for the MFEM-free RayFEM core: viewer launch and shutdown,
// and streaming of serialized meshes and fields. No C++ types are shared with
// other extensions (built in its own nanobind domain); MFEM meshes travel as
// text and fields as a space header plus a float64 array, so producers using
// any MFEM build, or none, can send to the viewer.
#include "libRayfem/init.hpp"
#include "libRayfem/network/stream.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace nb = nanobind;
using namespace nb::literals;

namespace {
// Coefficients are read in place: one-dimensional, contiguous float64 on the CPU.
using Values = nb::ndarray<const double, nb::ndim<1>, nb::c_contig, nb::device::cpu>;
using Field = std::tuple<std::string, std::string, Values>;

RayFEM::FieldData View(const Field& field) {
    const auto& values = std::get<2>(field);
    return {std::get<0>(field), std::get<1>(field), {values.data(), values.shape(0)}};
}

// A Python value for a schema field: bool is refused rather than read as int.
RayFEM::Value ToValue(const std::string& name, nb::handle value) {
    if (nb::isinstance<nb::bool_>(value)) throw nb::type_error(("Value " + name + " is a bool; send an int or float").c_str());
    if (nb::isinstance<nb::int_>(value)) return static_cast<int64_t>(nb::cast<int64_t>(value));
    if (nb::isinstance<nb::float_>(value)) return nb::cast<double>(value);
    if (nb::isinstance<nb::str>(value)) return nb::cast<std::string>(value);
    throw nb::type_error(("Value " + name + " must be an int, float or str").c_str());
}

RayFEM::ValueType ToType(const std::string& type) {
    if (type == "int") return RayFEM::ValueType::Integer;
    if (type == "float") return RayFEM::ValueType::Real;
    if (type == "str") return RayFEM::ValueType::Text;
    throw nb::value_error(("Unknown value type '" + type + "'; use 'int', 'float' or 'str'").c_str());
}

RayFEM::WindowParams Window(const std::string& host, int port, int control_port, int width, int height, int fps,
                            int history, int timeout_ms, const std::string& executable) {
    RayFEM::WindowParams p;
    p.host = host; p.port = port; p.control_port = control_port; p.width = width; p.height = height;
    p.fps = fps; p.history = history; p.timeout_ms = timeout_ms; p.executable = executable;
    return p;
}

void Check(bool ok, const RayFEM::StreamClient& client) {
    if (!ok) throw std::runtime_error(client.getLastError());
}
} // namespace

NB_MODULE(_rayfem, m) {
    m.doc() = "MFEM-free core of the RayFEM viewer client";

    nb::enum_<RayFEM::InitStatus>(m, "InitStatus")
        .value("Started", RayFEM::InitStatus::Started)
        .value("AlreadyRunning", RayFEM::InitStatus::AlreadyRunning)
        .value("Failed", RayFEM::InitStatus::Failed);
    nb::enum_<RayFEM::ShutdownStatus>(m, "ShutdownStatus")
        .value("Stopped", RayFEM::ShutdownStatus::Stopped)
        .value("NotRunning", RayFEM::ShutdownStatus::NotRunning)
        .value("Failed", RayFEM::ShutdownStatus::Failed);

    m.def("init", [](const std::string& host, int port, int control_port, int width, int height, int fps, int history,
                     int timeout_ms, const std::string& executable) {
            const auto params = Window(host, port, control_port, width, height, fps, history, timeout_ms, executable);
            RayFEM::InitResult result;
            {
                nb::gil_scoped_release release;
                result = RayFEM::init(params);
            }
            return std::make_tuple(result.status, result.executable, result.error);
        }, "host"_a = "127.0.0.1", "port"_a = 5555, "control_port"_a = 0, "width"_a = 1440, "height"_a = 900, "fps"_a = 60,
        "history"_a = 1000, "timeout_ms"_a = 5000, "executable"_a = "",
        "Start (or verify) a viewer. Returns (status, executable, error).");

    m.def("shutdown", [](const std::string& host, int port, int control_port, int timeout_ms) {
            RayFEM::WindowParams params;
            params.host = host; params.port = port; params.control_port = control_port; params.timeout_ms = timeout_ms;
            RayFEM::ShutdownResult result;
            {
                nb::gil_scoped_release release;
                result = RayFEM::shutdown(params);
            }
            return std::make_tuple(result.status, result.error);
        }, "host"_a = "127.0.0.1", "port"_a = 5555, "control_port"_a = 0, "timeout_ms"_a = 5000,
        "Ask a viewer to close. Returns (status, error).");

    m.def("stats", [](const std::string& host, int port, int control_port, int timeout_ms) {
            RayFEM::WindowParams params;
            params.host = host; params.port = port; params.control_port = control_port; params.timeout_ms = timeout_ms;
            RayFEM::ViewerStats result;
            {
                nb::gil_scoped_release release;
                result = RayFEM::stats(params);
            }
            return std::make_tuple(result.received, result.errors, result.dropped, result.skipped, result.error);
        }, "host"_a = "127.0.0.1", "port"_a = 5555, "control_port"_a = 0, "timeout_ms"_a = 2000,
        "Viewer counters: (received, errors, dropped, skipped, error).");

    nb::class_<RayFEM::StreamClient>(m, "StreamClient")
        .def(nb::init<const std::string&, size_t>(), "host"_a = "localhost", "port"_a = 5555)
        .def_prop_rw("binary_fields", &RayFEM::StreamClient::binaryFields, &RayFEM::StreamClient::setBinaryFields)
        .def_prop_ro("last_error", &RayFEM::StreamClient::getLastError)
        .def_prop_ro("connected", &RayFEM::StreamClient::connected)
        .def_prop_ro("address", &RayFEM::StreamClient::address)
        .def("start_queue", [](RayFEM::StreamClient& client, size_t max_bytes, const std::string& when_full) {
                RayFEM::StreamQueue options;
                options.max_bytes = max_bytes;
                if (when_full == "wait") options.when_full = RayFEM::StreamQueue::WhenFull::Wait;
                else if (when_full == "drop_oldest") options.when_full = RayFEM::StreamQueue::WhenFull::DropOldest;
                else if (when_full == "drop_newest") options.when_full = RayFEM::StreamQueue::WhenFull::DropNewest;
                else throw nb::value_error("when_full must be 'wait', 'drop_oldest' or 'drop_newest'");
                client.startQueue(options);
            }, "max_bytes"_a = size_t{512} << 20, "when_full"_a = "wait",
            "Send in the background; when_full is 'wait', 'drop_oldest' or 'drop_newest'.")
        .def("stop_queue", &RayFEM::StreamClient::stopQueue, "timeout_ms"_a = 5000, nb::call_guard<nb::gil_scoped_release>(),
             "Send what is queued (up to timeout_ms), then send in the caller's thread again.")
        .def_prop_ro("queued", &RayFEM::StreamClient::queued)
        .def("flush", &RayFEM::StreamClient::flush, "timeout_ms"_a = -1, nb::call_guard<nb::gil_scoped_release>(),
             "Wait until every packet is written to the network; False on timeout.")
        .def("queue_stats", [](const RayFEM::StreamClient& client) {
                const auto s = client.queueStats();
                return std::make_tuple(s.pending, s.pending_bytes, s.sent, s.dropped, s.failed);
            }, "(pending, pending_bytes, sent, dropped, failed)")
        .def("register_schema", [](RayFEM::StreamClient& client,
                                   const std::vector<std::tuple<std::string, std::string, bool, std::string, std::string>>& values,
                                   const std::vector<std::pair<std::string, std::string>>& tags, bool has_time,
                                   const std::string& primary,
                                   const std::vector<std::tuple<std::string, std::array<double, 3>, std::array<double, 3>, std::string>>& axes) {
                RayFEM::FrameSchema schema;
                for (const auto& [name, direction, origin, description] : axes)
                    schema.axes.push_back({name, direction, origin, description});
                for (const auto& [name, type, coordinate, unit, description] : values)
                    schema.values.push_back({name, ToType(type), coordinate, unit, description});
                for (const auto& [name, description] : tags) schema.tags.push_back({name, description});
                schema.has_time = has_time;
                schema.primary = primary;
                Check(client.registerSchema(schema), client);
            }, "values"_a, "tags"_a, "has_time"_a = true, "primary"_a = "", "axes"_a = std::vector<std::tuple<std::string, std::array<double, 3>, std::array<double, 3>, std::string>>{},
            "values: (name, 'int'|'float'|'str', coordinate, unit, description); tags: (name, description); "
            "axes: (name, direction, origin, description).")
        .def("send_mesh", [](RayFEM::StreamClient& client, uint64_t mesh_id, nb::bytes text) {
                const std::string_view view(text.c_str(), text.size());
                bool ok;
                {
                    nb::gil_scoped_release release;
                    ok = client.sendMesh(mesh_id, view);
                }
                Check(ok, client);
            }, "mesh_id"_a, "text"_a, "Register MFEM mesh text under mesh_id.")
        .def("send_frame", [](RayFEM::StreamClient& client, uint64_t mesh_id, const std::vector<Field>& fields,
                              uint32_t index, double time, const std::string& label, nb::dict values,
                              const std::vector<std::string>& tags, const std::optional<Field>& displacement) {
                RayFEM::FrameMetadata metadata;
                metadata.frame_index = index;
                metadata.sim_time = time;
                if (label.size() >= sizeof(metadata.label)) throw nb::value_error("Frame labels are at most 63 bytes");
                std::copy(label.begin(), label.end(), metadata.label);
                metadata.label[label.size()] = '\0';
                for (auto [key, value] : values) {
                    const std::string name = nb::cast<std::string>(key);
                    metadata.values.push_back({name, ToValue(name, value)});
                }
                metadata.tags = tags;
                std::vector<RayFEM::FieldData> data;
                data.reserve(fields.size());
                for (const auto& field : fields) data.push_back(View(field));
                const RayFEM::FieldData moved = displacement ? View(*displacement) : RayFEM::FieldData{};
                bool ok;
                {
                    nb::gil_scoped_release release;
                    ok = client.sendFrame(mesh_id, data, metadata, displacement ? &moved : nullptr);
                }
                Check(ok, client);
            }, "mesh_id"_a, "fields"_a, "index"_a, "time"_a = 0.0, "label"_a = "", "values"_a = nb::dict(),
            "tags"_a = std::vector<std::string>{}, "displacement"_a = nb::none(),
            "Send one frame: fields are (name, space header, float64 array).");
}
