#pragma once

#include "libRayfem/mesh/converter.hpp"
#include "raylib.h"
#include "raymath.h"
#include <algorithm>
#include <bit>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace RayFEM::UI {

// Surfaces are built as independent triangles, so every refined sample is
// repeated by each triangle that touches it (about six times on quad
// patches). Welding exact duplicates keeps one copy per sample on the GPU.
// raylib meshes use 16-bit indices, so vertices are split into chunks, each
// covering a contiguous range of the original triangles.
struct SurfaceChunk {
    std::vector<float> positions, normals;
    std::vector<unsigned char> colors;
    std::vector<double> values;           // per vertex, when the surface has field values
    std::vector<unsigned short> indices;  // three per triangle
    std::size_t first_triangle = 0;
};

struct IndexedSurface {
    std::vector<SurfaceChunk> chunks;
    std::size_t vertices = 0, triangles = 0;

    // The chunk and its local triangle for an original triangle index.
    std::pair<std::size_t, std::size_t> Locate(std::size_t triangle) const {
        auto chunk = std::upper_bound(chunks.begin(), chunks.end(), triangle,
            [](std::size_t value, const SurfaceChunk& c) { return value < c.first_triangle; });
        if (chunk == chunks.begin() || triangle >= triangles) throw std::out_of_range("Triangle is outside the surface");
        --chunk;
        return {std::size_t(chunk - chunks.begin()), triangle - chunk->first_triangle};
    }
};

inline IndexedSurface WeldSurface(const SurfaceData& surface, std::size_t max_chunk_vertices = 65535) {
    if (surface.positions.size() % 9 || surface.normals.size() != surface.positions.size() ||
        surface.colors.size() != surface.positions.size()/3*4)
        throw std::invalid_argument("Invalid surface buffers");
    const std::size_t count = surface.positions.size()/3;
    const bool values = surface.values.size() == count;
    max_chunk_vertices = std::clamp<std::size_t>(max_chunk_vertices, 3, 65535);
    IndexedSurface result;
    result.triangles = count/3;
    // Vertices match when every attribute matches bit for bit.
    struct Key {
        std::uint32_t bits[10];
        bool operator==(const Key& other) const { return std::memcmp(bits, other.bits, sizeof(bits)) == 0; }
    };
    struct Hash {
        std::size_t operator()(const Key& key) const {
            std::uint64_t hash = 1469598103934665603ULL;
            for (std::uint32_t word : key.bits) { hash ^= word; hash *= 1099511628211ULL; }
            return hash;
        }
    };
    auto key = [&](std::size_t vertex) {
        Key k{};
        for (int d = 0; d < 3; ++d) {
            k.bits[d] = std::bit_cast<std::uint32_t>(surface.positions[3*vertex+d]);
            k.bits[3+d] = std::bit_cast<std::uint32_t>(surface.normals[3*vertex+d]);
        }
        std::memcpy(&k.bits[6], &surface.colors[4*vertex], 4);
        const std::uint64_t value = values ? std::bit_cast<std::uint64_t>(surface.values[vertex]) : 0;
        k.bits[7] = static_cast<std::uint32_t>(value);
        k.bits[8] = static_cast<std::uint32_t>(value >> 32);
        return k;
    };
    std::unordered_map<Key, unsigned short, Hash> local;
    for (std::size_t triangle = 0; triangle < result.triangles; ++triangle) {
        // Start a chunk when this triangle might not fit in the current one.
        if (result.chunks.empty() || result.chunks.back().positions.size()/3 + 3 > max_chunk_vertices) {
            result.chunks.emplace_back().first_triangle = triangle;
            local.clear();
        }
        auto& chunk = result.chunks.back();
        for (int corner = 0; corner < 3; ++corner) {
            const std::size_t vertex = 3*triangle + corner;
            const auto [found, inserted] = local.try_emplace(key(vertex), static_cast<unsigned short>(chunk.positions.size()/3));
            if (inserted) {
                chunk.positions.insert(chunk.positions.end(), &surface.positions[3*vertex], &surface.positions[3*vertex] + 3);
                chunk.normals.insert(chunk.normals.end(), &surface.normals[3*vertex], &surface.normals[3*vertex] + 3);
                chunk.colors.insert(chunk.colors.end(), &surface.colors[4*vertex], &surface.colors[4*vertex] + 4);
                if (values) chunk.values.push_back(surface.values[vertex]);
            }
            chunk.indices.push_back(found->second);
        }
    }
    for (const auto& chunk : result.chunks) result.vertices += chunk.positions.size()/3;
    return result;
}

// Main thread only: one GPU mesh per chunk, all sharing the default material.
// CPU copies stay with each mesh for recoloring and selection overlays.
inline Model UploadIndexedSurface(const IndexedSurface& surface) {
    if (surface.chunks.empty()) return Model{};
    Model model{};
    model.transform = MatrixIdentity();
    model.meshCount = static_cast<int>(surface.chunks.size());
    model.meshes = static_cast<Mesh*>(MemAlloc(model.meshCount*sizeof(Mesh)));
    model.meshMaterial = static_cast<int*>(MemAlloc(model.meshCount*sizeof(int)));
    model.materialCount = 1;
    model.materials = static_cast<Material*>(MemAlloc(sizeof(Material)));
    if (!model.meshes || !model.meshMaterial || !model.materials) {
        MemFree(model.meshes); MemFree(model.meshMaterial); MemFree(model.materials);
        throw std::bad_alloc();
    }
    model.materials[0] = LoadMaterialDefault();
    for (int i = 0; i < model.meshCount; ++i) {
        const auto& chunk = surface.chunks[i];
        Mesh mesh{};
        mesh.vertexCount = static_cast<int>(chunk.positions.size()/3);
        mesh.triangleCount = static_cast<int>(chunk.indices.size()/3);
        mesh.vertices = static_cast<float*>(MemAlloc(chunk.positions.size()*sizeof(float)));
        mesh.normals = static_cast<float*>(MemAlloc(chunk.normals.size()*sizeof(float)));
        mesh.colors = static_cast<unsigned char*>(MemAlloc(chunk.colors.size()));
        mesh.indices = static_cast<unsigned short*>(MemAlloc(chunk.indices.size()*sizeof(unsigned short)));
        if (!mesh.vertices || !mesh.normals || !mesh.colors || !mesh.indices) {
            MemFree(mesh.vertices); MemFree(mesh.normals); MemFree(mesh.colors); MemFree(mesh.indices);
            model.meshCount = i;
            UnloadModel(model);
            throw std::bad_alloc();
        }
        std::copy(chunk.positions.begin(), chunk.positions.end(), mesh.vertices);
        std::copy(chunk.normals.begin(), chunk.normals.end(), mesh.normals);
        std::copy(chunk.colors.begin(), chunk.colors.end(), mesh.colors);
        std::copy(chunk.indices.begin(), chunk.indices.end(), mesh.indices);
        UploadMesh(&mesh, false);
        model.meshes[i] = mesh;
        model.meshMaterial[i] = 0;
    }
    return model;
}

} // namespace RayFEM::UI
