#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_set>
#include <variant>
#include <vector>

namespace RayFEM {
// What a client sends with its frames, registered once with
// Client::registerSchema before frames that use it.
enum class ValueType : uint8_t { Integer = 1, Real = 2, Text = 3 };
using Value = std::variant<int64_t, double, std::string>;
struct ValueField {
    std::string name;
    ValueType type = ValueType::Real;
    // A coordinate (integer or real) is a progress axis frames can be
    // labelled, ordered and filtered by, such as an iteration count. Other
    // values are shown with the frame, such as a virial ratio.
    bool coordinate = false;
    std::string unit, description;
};
struct TagField { std::string name, description; };
struct FrameSchema {
    std::vector<ValueField> values;
    std::vector<TagField> tags;
    // False when simulation time means nothing, as for a static model being
    // converged; viewers then label frames by the primary coordinate.
    bool has_time = true;
    // The coordinate frames are labelled and plotted by; empty uses time.
    std::string primary;
    const ValueField* find(std::string_view name) const {
        for (const auto& v : values) if (v.name == name) return &v;
        return nullptr;
    }
    bool hasTag(std::string_view name) const {
        return std::any_of(tags.begin(), tags.end(), [&](const auto& t) { return t.name == name; });
    }
};
struct FrameValue { std::string name; Value value; };

struct FrameMetadata {
    uint32_t frame_index = 0;
    double sim_time = 0;
    char label[64] = "MFEM Simulation";
    // Declared in the registered schema: values (any subset, each at most
    // once) and tags such as "converged".
    std::vector<FrameValue> values;
    std::vector<std::string> tags;

    const Value* find(std::string_view name) const {
        for (const auto& v : values) if (v.name == name) return &v.value;
        return nullptr;
    }
    // Integer or real values as a double.
    std::optional<double> number(std::string_view name) const {
        const Value* v = find(name);
        if (!v) return std::nullopt;
        if (const auto* i = std::get_if<int64_t>(v)) return static_cast<double>(*i);
        if (const auto* d = std::get_if<double>(v)) return *d;
        return std::nullopt;
    }
    bool hasTag(std::string_view tag) const { return std::find(tags.begin(), tags.end(), tag) != tags.end(); }
};

// The wire format is explicitly encoded, never a memcpy of a C++ struct.
namespace Protocol {
inline constexpr uint16_t Version = 2;
inline constexpr size_t HeaderBytes = 128;
inline constexpr size_t MaxPayloadBytes = 64 * 1024 * 1024;
inline constexpr size_t MaxFields = 64;
inline constexpr size_t MaxNameBytes = 255;
inline constexpr uint32_t HasDisplacement = 1;
// Fields (and displacement) use the binary layout below instead of MFEM text:
// "RFB1", u32 header length, the FiniteElementSpace::Save header text,
// u64 value count, then each value as the u64 bits of an IEEE double. All
// integers are big-endian, like the packet header.
inline constexpr uint32_t BinaryFields = 2;
inline constexpr std::string_view BinaryFieldMagic{"RFB1", 4};
inline constexpr size_t MaxSpaceHeaderBytes = 1024 * 1024;
// A frame payload ends with a descriptor (its schema values and tags).
inline constexpr uint32_t HasDescriptor = 4;
inline constexpr size_t MaxSchemaEntries = 64;
inline constexpr size_t MaxTextBytes = 1024;
inline constexpr std::string_view SchemaMagic{"RFS1", 4};
enum class Kind : uint16_t { Mesh = 1, Frame = 2, Schema = 3 };

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
    if (value < 1 || value > 3) return fail("Unknown RayFEM packet type");
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
    if (header.flags & ~(HasDisplacement | BinaryFields | HasDescriptor)) return fail("Unknown packet flags");
    if (!std::isfinite(header.metadata.sim_time)) return fail("Non-finite frame time");
    if (header.payload_bytes > MaxPayloadBytes || header.payload_bytes != data.size() - HeaderBytes)
        return fail("Invalid or oversized payload length");
    if (header.kind == Kind::Mesh && (header.field_count != 0 || header.flags != 0 || !header.payload_bytes))
        return fail("Invalid mesh packet");
    if (header.kind == Kind::Frame && (header.field_count == 0 || header.field_count > MaxFields))
        return fail("Invalid field count");
    if (header.kind == Kind::Schema && (header.field_count != 0 || header.flags != 0 || !header.payload_bytes))
        return fail("Invalid schema packet");
    return true;
}

// ---- Schema and frame descriptors ----------------------------------------
// Strings are a u16 length then bytes; integers are big-endian like the header.

inline void appendText(std::string& out, std::string_view text) {
    appendInteger(out, text.size(), 2);
    out.append(text);
}
inline bool readText(std::string_view data, size_t& offset, size_t limit, std::string& text) {
    uint64_t size = 0;
    if (!readInteger(data, offset, 2, size) || size > limit || size > data.size() - offset) return false;
    text.assign(data.data() + offset, size);
    offset += size;
    return true;
}
inline bool validName(std::string_view name) {
    return !name.empty() && name.size() <= MaxNameBytes && name.find('\0') == std::string_view::npos;
}

// Names unique and well formed, coordinates numeric, the primary a coordinate.
inline bool validateSchema(const FrameSchema& schema, std::string& error) {
    auto fail = [&](std::string message) { error = std::move(message); return false; };
    if (schema.values.size() > MaxSchemaEntries || schema.tags.size() > MaxSchemaEntries)
        return fail("A schema declares at most 64 values and 64 tags");
    std::unordered_set<std::string_view> names, tags;
    for (const auto& v : schema.values) {
        if (!validName(v.name) || !names.insert(v.name).second) return fail("Schema value names must be unique, nonempty and at most 255 bytes");
        if (v.type != ValueType::Integer && v.type != ValueType::Real && v.type != ValueType::Text) return fail("Unknown value type for " + v.name);
        if (v.coordinate && v.type == ValueType::Text) return fail("Coordinate " + v.name + " must be an integer or real value");
        if (v.unit.size() > MaxNameBytes || v.description.size() > MaxTextBytes) return fail("Unit or description too long for " + v.name);
    }
    for (const auto& t : schema.tags) {
        if (!validName(t.name) || !tags.insert(t.name).second) return fail("Schema tag names must be unique, nonempty and at most 255 bytes");
        if (t.description.size() > MaxTextBytes) return fail("Description too long for tag " + t.name);
    }
    if (!schema.primary.empty()) {
        const auto* primary = schema.find(schema.primary);
        if (!primary || !primary->coordinate) return fail("The primary coordinate must be a declared coordinate");
    }
    return true;
}
inline std::string encodeSchema(const FrameSchema& schema) {
    std::string out(SchemaMagic);
    appendInteger(out, schema.has_time ? 1 : 0, 1);
    appendText(out, schema.primary);
    appendInteger(out, schema.values.size(), 2);
    for (const auto& v : schema.values) {
        appendInteger(out, static_cast<uint8_t>(v.type), 1);
        appendInteger(out, v.coordinate ? 1 : 0, 1);
        appendText(out, v.name); appendText(out, v.unit); appendText(out, v.description);
    }
    appendInteger(out, schema.tags.size(), 2);
    for (const auto& t : schema.tags) { appendText(out, t.name); appendText(out, t.description); }
    return out;
}
inline bool decodeSchema(std::string_view data, FrameSchema& schema, std::string& error) {
    auto fail = [&](const char* message) { error = message; return false; };
    if (!data.starts_with(SchemaMagic)) return fail("Invalid schema magic");
    size_t pos = SchemaMagic.size();
    uint64_t value = 0, count = 0;
    if (!readInteger(data, pos, 1, value) || value > 1) return fail("Invalid schema flags");
    schema = {};
    schema.has_time = value == 1;
    if (!readText(data, pos, MaxNameBytes, schema.primary) || !readInteger(data, pos, 2, count) || count > MaxSchemaEntries)
        return fail("Truncated or oversized schema");
    for (uint64_t i = 0; i < count; ++i) {
        ValueField v;
        uint64_t type = 0, coordinate = 0;
        if (!readInteger(data, pos, 1, type) || !readInteger(data, pos, 1, coordinate) || coordinate > 1 ||
            !readText(data, pos, MaxNameBytes, v.name) || !readText(data, pos, MaxNameBytes, v.unit) ||
            !readText(data, pos, MaxTextBytes, v.description)) return fail("Truncated schema value");
        v.type = static_cast<ValueType>(type);
        v.coordinate = coordinate == 1;
        schema.values.push_back(std::move(v));
    }
    if (!readInteger(data, pos, 2, count) || count > MaxSchemaEntries) return fail("Truncated or oversized schema tags");
    for (uint64_t i = 0; i < count; ++i) {
        TagField t;
        if (!readText(data, pos, MaxNameBytes, t.name) || !readText(data, pos, MaxTextBytes, t.description)) return fail("Truncated schema tag");
        schema.tags.push_back(std::move(t));
    }
    if (pos != data.size()) return fail("Unexpected bytes after schema");
    return validateSchema(schema, error);
}

// A frame's values and tags by schema index. Integers given for real values
// are widened; everything else must match its declared type.
inline bool encodeDescriptor(const FrameSchema& schema, const FrameMetadata& metadata, std::string& out, std::string& error) {
    auto fail = [&](std::string message) { error = std::move(message); return false; };
    if (metadata.values.size() > MaxSchemaEntries || metadata.tags.size() > MaxSchemaEntries) return fail("Too many frame values or tags");
    std::unordered_set<std::string_view> seen;
    appendInteger(out, metadata.values.size(), 2);
    for (const auto& v : metadata.values) {
        const auto* field = schema.find(v.name);
        if (!field) return fail("Value " + v.name + " is not in the registered schema");
        if (!seen.insert(v.name).second) return fail("Value " + v.name + " is given twice");
        appendInteger(out, static_cast<size_t>(field - schema.values.data()), 2);
        switch (field->type) {
            case ValueType::Integer:
                if (!std::holds_alternative<int64_t>(v.value)) return fail("Value " + v.name + " must be an integer");
                appendInteger(out, std::bit_cast<uint64_t>(std::get<int64_t>(v.value)), 8);
                break;
            case ValueType::Real: {
                double number = 0;
                if (const auto* i = std::get_if<int64_t>(&v.value)) number = static_cast<double>(*i);
                else if (const auto* d = std::get_if<double>(&v.value)) number = *d;
                else return fail("Value " + v.name + " must be a number");
                if (!std::isfinite(number)) return fail("Value " + v.name + " must be finite");
                appendInteger(out, std::bit_cast<uint64_t>(number), 8);
                break;
            }
            case ValueType::Text: {
                const auto* text = std::get_if<std::string>(&v.value);
                if (!text) return fail("Value " + v.name + " must be text");
                if (text->size() > MaxTextBytes) return fail("Text value " + v.name + " exceeds 1024 bytes");
                appendText(out, *text);
                break;
            }
        }
    }
    std::unordered_set<std::string_view> tags;
    appendInteger(out, metadata.tags.size(), 2);
    for (const auto& tag : metadata.tags) {
        const auto found = std::find_if(schema.tags.begin(), schema.tags.end(), [&](const auto& t) { return t.name == tag; });
        if (found == schema.tags.end()) return fail("Tag " + tag + " is not in the registered schema");
        if (!tags.insert(tag).second) return fail("Tag " + tag + " is given twice");
        appendInteger(out, static_cast<size_t>(found - schema.tags.begin()), 2);
    }
    return true;
}
inline bool decodeDescriptor(const FrameSchema& schema, std::string_view data, FrameMetadata& metadata, std::string& error) {
    auto fail = [&](const char* message) { error = message; return false; };
    size_t pos = 0;
    uint64_t count = 0, index = 0, bits = 0;
    metadata.values.clear(); metadata.tags.clear();
    std::vector<bool> seen(schema.values.size()), tagged(schema.tags.size());
    if (!readInteger(data, pos, 2, count) || count > schema.values.size()) return fail("Invalid frame value count");
    for (uint64_t i = 0; i < count; ++i) {
        if (!readInteger(data, pos, 2, index) || index >= schema.values.size() || seen[index]) return fail("Invalid or repeated frame value");
        seen[index] = true;
        const auto& field = schema.values[index];
        FrameValue v{field.name, int64_t{0}};
        if (field.type == ValueType::Text) {
            std::string text;
            if (!readText(data, pos, MaxTextBytes, text)) return fail("Truncated text value");
            v.value = std::move(text);
        } else {
            if (!readInteger(data, pos, 8, bits)) return fail("Truncated numeric value");
            if (field.type == ValueType::Integer) v.value = std::bit_cast<int64_t>(bits);
            else {
                const double number = std::bit_cast<double>(bits);
                if (!std::isfinite(number)) return fail("Non-finite frame value");
                v.value = number;
            }
        }
        metadata.values.push_back(std::move(v));
    }
    if (!readInteger(data, pos, 2, count) || count > schema.tags.size()) return fail("Invalid frame tag count");
    for (uint64_t i = 0; i < count; ++i) {
        if (!readInteger(data, pos, 2, index) || index >= schema.tags.size() || tagged[index]) return fail("Invalid or repeated frame tag");
        tagged[index] = true;
        metadata.tags.push_back(schema.tags[index].name);
    }
    if (pos != data.size()) return fail("Unexpected bytes after frame descriptor");
    return true;
}
} // namespace Protocol
} // namespace RayFEM
