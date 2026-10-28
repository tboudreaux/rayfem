#pragma once

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>

namespace RayFEM {
struct FrameMetadata {
    uint32_t frame_index = 0;
    double sim_time = 0;
    char label[64] = "MFEM Simulation";
};

// The wire format is explicitly encoded, never a memcpy of a C++ struct.
namespace Protocol {
inline constexpr uint16_t Version = 2;
inline constexpr size_t HeaderBytes = 128;
inline constexpr size_t MaxPayloadBytes = 64 * 1024 * 1024;
inline constexpr size_t MaxFields = 64;
inline constexpr size_t MaxNameBytes = 255;
inline constexpr uint32_t HasDisplacement = 1;
enum class Kind : uint16_t { Mesh = 1, Frame = 2 };

struct Header {
    Kind kind = Kind::Frame;
    uint32_t flags = 0;
    uint64_t mesh_id = 0;
    uint64_t session_id = 0;
    uint64_t mesh_revision = 0;
    uint32_t field_count = 0;
    uint64_t payload_bytes = 0;
    FrameMetadata metadata;
};

inline void appendInteger(std::string& out, uint64_t value, size_t bytes) {
    for (size_t i = bytes; i > 0; --i) out.push_back(static_cast<char>(value >> ((i - 1) * 8)));
}
inline bool readInteger(std::string_view data, size_t& offset, size_t bytes, uint64_t& value) {
    if (offset > data.size() || bytes > data.size() - offset) return false;
    value = 0;
    for (size_t i = 0; i < bytes; ++i) value = (value << 8) | static_cast<unsigned char>(data[offset++]);
    return true;
}
inline std::string encodeHeader(const Header& header) {
    std::string out("RAYFEM2\0", 8);
    appendInteger(out, Version, 2);
    appendInteger(out, static_cast<uint16_t>(header.kind), 2);
    appendInteger(out, header.flags, 4);
    appendInteger(out, header.mesh_id, 8);
    appendInteger(out, header.session_id, 8);
    appendInteger(out, header.mesh_revision, 8);
    appendInteger(out, header.metadata.frame_index, 4);
    appendInteger(out, header.field_count, 4);
    appendInteger(out, std::bit_cast<uint64_t>(header.metadata.sim_time), 8);
    appendInteger(out, header.payload_bytes, 8);
    out.append(header.metadata.label, sizeof(header.metadata.label));
    return out;
}
inline bool decodeHeader(std::string_view data, Header& header, std::string& error) {
    auto fail = [&](const char* message) { error = message; return false; };
    if (data.size() < HeaderBytes || data.substr(0, 8) != std::string_view("RAYFEM2\0", 8))
        return fail("Invalid RayFEM packet magic or truncated header");
    size_t pos = 8;
    uint64_t value = 0;
    readInteger(data, pos, 2, value);
    if (value != Version) return fail("Unsupported RayFEM protocol version");
    readInteger(data, pos, 2, value);
    if (value != 1 && value != 2) return fail("Unknown RayFEM packet type");
    header.kind = static_cast<Kind>(value);
    readInteger(data, pos, 4, value); header.flags = static_cast<uint32_t>(value);
    readInteger(data, pos, 8, header.mesh_id);
    readInteger(data, pos, 8, header.session_id);
    readInteger(data, pos, 8, header.mesh_revision);
    readInteger(data, pos, 4, value); header.metadata.frame_index = static_cast<uint32_t>(value);
    readInteger(data, pos, 4, value); header.field_count = static_cast<uint32_t>(value);
    readInteger(data, pos, 8, value); header.metadata.sim_time = std::bit_cast<double>(value);
    readInteger(data, pos, 8, header.payload_bytes);
    std::memcpy(header.metadata.label, data.data() + pos, sizeof(header.metadata.label));
    header.metadata.label[sizeof(header.metadata.label) - 1] = '\0';
    if (!header.session_id || !header.mesh_revision) return fail("Missing stream identity or mesh revision");
    if (header.flags & ~HasDisplacement) return fail("Unknown packet flags");
    if (!std::isfinite(header.metadata.sim_time)) return fail("Non-finite frame time");
    if (header.payload_bytes > MaxPayloadBytes || header.payload_bytes != data.size() - HeaderBytes)
        return fail("Invalid or oversized payload length");
    if (header.kind == Kind::Mesh && (header.field_count != 0 || header.flags != 0 || !header.payload_bytes))
        return fail("Invalid mesh packet");
    if (header.kind == Kind::Frame && (header.field_count == 0 || header.field_count > MaxFields))
        return fail("Invalid field count");
    return true;
}
} // namespace Protocol
} // namespace RayFEM
