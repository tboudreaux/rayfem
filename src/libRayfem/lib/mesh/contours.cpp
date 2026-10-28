#include "libRayfem/mesh/contours.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace RayFEM {
std::vector<double> ContourLevels(const ColorScaleMapping& mapping, int count) {
    std::vector<double> levels;
    if (count <= 0) return levels;
    levels.reserve(count);
    for (int i = 1; i <= count; ++i) {
        const double level = ColorScaleValue(double(i)/(count + 1), mapping);
        if (std::isfinite(level)) levels.push_back(level);
    }
    std::sort(levels.begin(), levels.end());
    levels.erase(std::unique(levels.begin(), levels.end()), levels.end());
    return levels;
}

std::vector<LineSegment> ExtractContours(std::span<const float> positions, std::span<const float> values,
                                         std::span<const double> levels) {
    if (positions.size() % 9 || values.size()*3 != positions.size())
        throw std::invalid_argument("Contours require one value per triangle-soup vertex");
    if (!std::is_sorted(levels.begin(), levels.end()))
        throw std::invalid_argument("Contour levels must be sorted");
    std::vector<LineSegment> lines;
    if (levels.empty()) return lines;
    for (std::size_t t = 0; t < values.size()/3; ++t) {
        const float* p = positions.data() + 9*t;
        const float* v = values.data() + 3*t;
        if (!std::isfinite(v[0]) || !std::isfinite(v[1]) || !std::isfinite(v[2])) continue;
        const double low = std::min({v[0], v[1], v[2]}), high = std::max({v[0], v[1], v[2]});
        if (!(high > low)) continue;  // a flat triangle has no isoline crossing
        auto vertex = [&](int i) { return Vector3{p[3*i], p[3*i + 1], p[3*i + 2]}; };
        // A vertex is "above" at or over the level, so each level crosses
        // exactly two edges of a triangle that straddles it.
        for (auto level = std::lower_bound(levels.begin(), levels.end(), low);
             level != levels.end() && *level <= high; ++level) {
            Vector3 ends[2];
            int found = 0;
            for (int i = 0; i < 3; ++i) {
                const int j = (i + 1) % 3;
                if ((v[i] >= *level) == (v[j] >= *level)) continue;
                const double s = std::clamp((*level - v[i])/(double(v[j]) - v[i]), 0.0, 1.0);
                const Vector3 a = vertex(i), b = vertex(j);
                ends[found++] = {float(a.x + s*(b.x - a.x)), float(a.y + s*(b.y - a.y)), float(a.z + s*(b.z - a.z))};
            }
            if (found != 2) continue;
            if (ends[0].x == ends[1].x && ends[0].y == ends[1].y && ends[0].z == ends[1].z) continue;
            lines.push_back({ends[0], ends[1]});
        }
    }
    return lines;
}
} // namespace RayFEM
