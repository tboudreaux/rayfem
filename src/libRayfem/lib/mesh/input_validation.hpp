#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace RayFEM::detail {
// The bundled MFEM knot evaluator uses fixed arrays for degree <= 10. Its
// release builds do not guard CalcShape, so reject larger wire degrees before
// constructing or evaluating a NURBS collection.
constexpr int MaxNURBSOrder = 10;
inline void validateCollectionOrder(std::string_view name) {
    for (size_t p = name.find("_P"); p != std::string::npos; p = name.find("_P", p + 2)) {
        std::istringstream input(std::string{name.substr(p + 2)});
        int order = 0;
        if (!(input >> order) || order < 0 || order > 32)
            throw std::runtime_error("Polynomial order exceeds supported limit (32)");
    }
    if (!name.starts_with("NURBS")) return;
    const size_t prefix = name.starts_with("NURBS_HCurl") ? 11 : name.starts_with("NURBS_HDiv") ? 10 : 5;
    if (name.size() == prefix) return; // Variable degree; extended header follows.
    std::istringstream input(std::string{name.substr(prefix)});
    int order = 0;
    std::string extra;
    if (!(input >> order) || (input >> extra) || order < 0 || order > MaxNURBSOrder)
        throw std::runtime_error("NURBS order exceeds supported limit (10)");
}

// Check the common serial/NURBS topology before handing it to MFEM: its
// optimized reader assumes dimensions and connectivity indexes are valid.
inline void validateMeshText(const std::string& data) {
    // Native readers allocate these tables before checking whether enough
    // records follow. This also covers NC formats whose connectivity grammar
    // is validated by MFEM itself below.
    {
        std::istringstream sections(data);std::string section;
        while(std::getline(sections,section)) {
            const auto first=section.find_first_not_of(" \t\r");
            if(first==std::string::npos||section[first]=='#')continue;
            section=section.substr(first,section.find_last_not_of(" \t\r")-first+1);
            if(section=="dimension") {
                int dimension=0;
                if(!(sections>>dimension)||dimension<1||dimension>3)throw std::runtime_error("Invalid mesh dimension");
            } else if(section=="elements"||section=="boundary"||section=="vertices"||section=="vertex_parents"||section=="coarse_elements"||section=="root_state"||section=="edges") {
                long long count=0;
                if(!(sections>>count)||count<0||count>1000000||static_cast<unsigned long long>(count)>data.size())
                    throw std::runtime_error("Invalid or excessive native mesh table count");
            } else if(section.starts_with("FiniteElementCollection: ")) {
                validateCollectionOrder(std::string_view(section).substr(25));
            } else if(section.starts_with("VDim:")) {
                std::istringstream property(section.substr(5));int dimension=0;
                if(!(property>>dimension)||dimension<1||dimension>3)throw std::runtime_error("Invalid mesh node dimension");
            } else if(section.starts_with("Ordering:")) {
                std::istringstream property(section.substr(9));int ordering=-1;
                if(!(property>>ordering)||ordering<0||ordering>1)throw std::runtime_error("Invalid mesh node ordering");
            }
        }
    }
    std::istringstream input(data);
    std::string line;
    std::getline(input, line);
    if (line.starts_with("MFEM NC mesh") || line == "MFEM mesh v1.1") return;
    const bool nurbs = line.starts_with("MFEM NURBS mesh");
    int dimension = 0, vertices = -1, largest_vertex = -1;
    int knot_vectors = -1, largest_knot = -1;
    bool patches = false;
    bool has_elements = false, has_boundary = false;
    constexpr int vertex_counts[] = {1, 2, 3, 4, 4, 8, 6, 5};
    auto integer = [&]() {
        int value = 0;
        if (!(input >> value)) throw std::runtime_error("Truncated mesh topology");
        return value;
    };
    while (std::getline(input, line)) {
        const auto begin = line.find_first_not_of(" \t\r");
        if (begin == std::string::npos || line[begin] == '#') continue;
        line = line.substr(begin, line.find_last_not_of(" \t\r") - begin + 1);
        if (line == "dimension") {
            dimension = integer();
            if (dimension < 1 || dimension > 3) throw std::runtime_error("Invalid mesh dimension");
        } else if (line == "elements" || line == "boundary") {
            const int count = integer();
            if (count < 0 || static_cast<size_t>(count) > data.size() / 4)
                throw std::runtime_error("Invalid mesh element count");
            if (line == "elements") { has_elements = count > 0; } else { has_boundary = true; }
            for (int i = 0; i < count; ++i) {
                integer(); // attribute
                const int geometry = integer();
                if (geometry < 0 || geometry > 7) throw std::runtime_error("Invalid mesh element geometry");
                for (int j = 0; j < vertex_counts[geometry]; ++j) {
                    const int vertex = integer();
                    if (vertex < 0) throw std::runtime_error("Negative mesh vertex index");
                    largest_vertex = std::max(largest_vertex, vertex);
                }
            }
        } else if (nurbs && line == "edges") {
            const int count = integer();
            if (count < 0 || static_cast<size_t>(count) > data.size() / 6)
                throw std::runtime_error("Invalid NURBS edge count");
            for (int i = 0; i < count; ++i) {
                const int knot = integer(), first = integer(), second = integer();
                if (knot < 0 || first < 0 || second < 0)
                    throw std::runtime_error("Invalid NURBS edge index");
                largest_knot = std::max(largest_knot, knot);
                largest_vertex = std::max({largest_vertex, first, second});
            }
        } else if (nurbs && line == "patches") {
            patches = true;
        } else if (nurbs && line == "knotvectors") {
            const int count = integer();
            if (count < 1 || static_cast<size_t>(count) > data.size() / 4 || (patches && count > 3))
                throw std::runtime_error("Invalid NURBS knot-vector count");
            if (!patches) knot_vectors = count;
            for (int i = 0; i < count; ++i) {
                const int order = integer(), control_points = integer();
                if (order < 1 || order > MaxNURBSOrder || control_points <= order ||
                    static_cast<size_t>(control_points) + order + 1 > data.size())
                    throw std::runtime_error("Invalid NURBS degree or control-point count");
                double previous = -std::numeric_limits<double>::infinity();
                double first_active = 0, last_active = 0;
                for (int j = 0; j < control_points + order + 1; ++j) {
                    double knot = 0;
                    if (!(input >> knot) || !std::isfinite(knot) || knot < previous)
                        throw std::runtime_error("Invalid or truncated NURBS knot vector");
                    previous = knot;
                    if (j == order) first_active = knot;
                    if (j == control_points) last_active = knot;
                }
                if (!(last_active > first_active)) throw std::runtime_error("NURBS knot vector has an empty domain");
            }
        } else if (line == "vertices") {
            vertices = integer();
            if (vertices < 1 || static_cast<size_t>(vertices) > data.size())
                throw std::runtime_error("Invalid mesh vertex count");
            if (!nurbs) {
                std::string geometry;
                if (!(input >> geometry)) throw std::runtime_error("Missing mesh vertex coordinates");
                if (geometry != "nodes") {
                    std::istringstream value(geometry);
                    int space_dimension = 0;
                    if (!(value >> space_dimension) || space_dimension < dimension || space_dimension > 3)
                        throw std::runtime_error("Invalid mesh space dimension");
                    for (int i = 0; i < vertices; ++i) for (int j = 0; j < space_dimension; ++j) {
                        double coordinate = 0;
                        if (!(input >> coordinate) || !std::isfinite(coordinate))
                            throw std::runtime_error("Invalid mesh vertex coordinates");
                    }
                }
            }
        } else if (line.starts_with("FiniteElementCollection: ")) {
            validateCollectionOrder(std::string_view(line).substr(25));
        } else if (line.starts_with("VDim:")) {
            std::istringstream property(line.substr(5));
            int vdim = 0;
            if (!(property >> vdim) || vdim < dimension || vdim > 3)
                throw std::runtime_error("Invalid mesh node dimension");
        }
    }
    if (!has_elements || !has_boundary || !dimension || vertices < 1 || largest_vertex >= vertices)
        throw std::runtime_error("Invalid mesh connectivity or missing topology sections");
    if (nurbs && !patches && (knot_vectors < 1 || largest_knot >= knot_vectors))
        throw std::runtime_error("NURBS edge references an invalid knot vector");
}

} // namespace RayFEM::detail
