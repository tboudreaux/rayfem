#include "libRayfem/profile/surface_shape.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <numbers>
#include <sstream>
#include <stdexcept>

namespace {
constexpr double missing = std::numeric_limits<double>::quiet_NaN();
std::string Quote(const std::string& text) { std::string result="\"";for(char c:text){if(c=='\"')result+='\"';result+=c;}return result+'\"'; }
double Radius(const RayFEM::SurfaceTraceSample& s) { return s.valid && std::isfinite(s.ray_distance) ? s.ray_distance : missing; }
} // namespace

namespace RayFEM {
std::vector<SurfaceTraceRay> MakeSurfaceShapeRays(const SurfaceShapeOptions& o) {
    if (o.colatitudes < 3 || o.meridians < 1 || o.equator < 3)
        throw std::invalid_argument("A surface shape needs at least 3 colatitudes, 1 meridian and 3 equatorial rays");
    constexpr double pi = std::numbers::pi;
    std::vector<SurfaceTraceRay> rays;
    rays.reserve(static_cast<std::size_t>(o.meridians)*o.colatitudes + o.equator);
    for (int m = 0; m < o.meridians; ++m) {
        auto meridian = MakeMeridianRays(o.center, o.axis, o.reference, 2*pi*m/o.meridians, 0, pi, o.colatitudes);
        rays.insert(rays.end(), meridian.begin(), meridian.end());
    }
    // Without the closing ray, which repeats the first.
    auto ring = MakeAzimuthRays(o.center, o.axis, o.reference, pi/2, 0, 2*pi*(o.equator - 1)/o.equator, o.equator);
    rays.insert(rays.end(), ring.begin(), ring.end());
    return rays;
}

SurfaceShape MeasureSurfaceShape(const SurfaceShapeOptions& o, const std::vector<SurfaceTraceSample>& samples) {
    const std::size_t meridianRays = static_cast<std::size_t>(o.meridians)*o.colatitudes;
    if (samples.size() != meridianRays + o.equator) throw std::invalid_argument("Surface shape samples do not match its rays");
    SurfaceShape shape;
    shape.rays = samples.size();
    for (const auto& s : samples) if (std::isfinite(Radius(s))) {
        ++shape.hits;
        if (std::isfinite(s.residual)) shape.max_residual = std::max(shape.max_residual, s.residual);
    }
    // Each colatitude across the meridians; the volume integrates the mean
    // of r^3 over the sphere of directions.
    std::vector<double> cubes(o.colatitudes, missing);
    for (int c = 0; c < o.colatitudes; ++c) {
        SurfaceShapeSample p;
        p.colatitude = std::numbers::pi*c/(o.colatitudes - 1);
        double sum = 0, cube = 0;
        for (int m = 0; m < o.meridians; ++m) {
            const double r = Radius(samples[static_cast<std::size_t>(m)*o.colatitudes + c]);
            if (!std::isfinite(r)) continue;
            p.min = p.hits ? std::min(p.min, r) : r;
            p.max = p.hits ? std::max(p.max, r) : r;
            sum += r; cube += r*r*r; ++p.hits;
        }
        if (p.hits) { p.mean = sum/p.hits; cubes[c] = cube/p.hits; }
        shape.profile.push_back(p);
    }
    shape.north = shape.profile.front().mean;
    shape.south = shape.profile.back().mean;
    if (std::isfinite(shape.north) && std::isfinite(shape.south)) shape.polar = (shape.north + shape.south)/2;
    else shape.polar = std::isfinite(shape.north) ? shape.north : shape.south;
    // Trapezoids in colatitude: R_v^3 = 1/2 integral of <r^3> sin(theta).
    double integral = 0;
    bool complete = true;
    for (int c = 1; c < o.colatitudes; ++c) {
        if (!std::isfinite(cubes[c - 1]) || !std::isfinite(cubes[c])) { complete = false; break; }
        const double a = shape.profile[c - 1].colatitude, b = shape.profile[c].colatitude;
        integral += (cubes[c - 1]*std::sin(a) + cubes[c]*std::sin(b))*(b - a)/2;
    }
    if (complete) shape.volume_radius = std::cbrt(integral/2);

    double sum = 0;
    int hits = 0;
    for (int e = 0; e < o.equator; ++e) {
        const auto& s = samples[meridianRays + e];
        const double r = Radius(s);
        shape.equator_azimuth.push_back(s.parameter);
        shape.equator_radius.push_back(r);
        if (!std::isfinite(r)) continue;
        shape.equatorial_min = hits ? std::min(shape.equatorial_min, r) : r;
        shape.equatorial_max = hits ? std::max(shape.equatorial_max, r) : r;
        sum += r; ++hits;
    }
    if (hits) shape.equatorial = sum/hits;
    if (std::isfinite(shape.polar) && shape.equatorial > 0) {
        shape.axis_ratio = shape.polar/shape.equatorial;
        shape.flattening = 1 - shape.axis_ratio;
    }
    if (shape.equatorial_max > 0) shape.equatorial_ellipticity = 1 - shape.equatorial_min/shape.equatorial_max;
    return shape;
}

SurfaceShape SampleSurfaceShape(const SurfaceTraceRegistry& registry, mfem::Mesh& mesh, mfem::GridFunction& field,
                                const SurfaceShapeOptions& options, const RenderOptions& render,
                                std::vector<SurfaceTraceSample>* out) {
    const auto rays = MakeSurfaceShapeRays(options);
    std::vector<SurfaceTraceSample> samples;
    samples.reserve(rays.size());
    // The trace refuses calls whose rays times faces exceed its limits; fewer
    // rays per call fit, at the same total cost.
    std::size_t batch = rays.size();
    for (std::size_t begin = 0; begin < rays.size();) {
        const std::size_t end = std::min(rays.size(), begin + batch);
        try {
            const auto part = registry.sample("curved", mesh, field, {rays.begin() + begin, rays.begin() + end}, render);
            samples.insert(samples.end(), part.begin(), part.end());
            begin = end;
        } catch (const std::invalid_argument& error) {
            const std::string message = error.what();
            const bool tooLarge = message.find("too large") != std::string::npos || message.find("exceeds") != std::string::npos;
            if (!tooLarge || batch == 1) throw;
            batch = std::max<std::size_t>(1, batch/4);
        }
    }
    auto shape = MeasureSurfaceShape(options, samples);
    if (out) *out = std::move(samples);
    return shape;
}

void ExportSurfaceShapeCSV(std::ostream& stream, const SurfaceShape& shape, const ProfileMetadata& metadata) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(std::numeric_limits<double>::max_digits10);
    out << "# field " << Quote(metadata.field_name) << ", frame " << metadata.frame << ", time " << metadata.time << '\n'
        << "# equatorial_radius " << shape.equatorial << ", polar_radius " << shape.polar << ", north " << shape.north
        << ", south " << shape.south << ", flattening " << shape.flattening << ", axis_ratio " << shape.axis_ratio
        << ", volume_radius " << shape.volume_radius << ", equatorial_ellipticity " << shape.equatorial_ellipticity << '\n'
        << "kind,angle,radius_mean,radius_min,radius_max,hits\n";
    for (const auto& p : shape.profile)
        out << "colatitude," << p.colatitude << ',' << p.mean << ',' << p.min << ',' << p.max << ',' << p.hits << '\n';
    for (std::size_t i = 0; i < shape.equator_radius.size(); ++i) {
        const double r = shape.equator_radius[i];
        out << "equator_azimuth," << shape.equator_azimuth[i] << ',' << r << ',' << r << ',' << r << ',' << int(std::isfinite(r)) << '\n';
    }
    stream << out.str();
    if (!stream) throw std::runtime_error("Could not write surface shape CSV");
}
} // namespace RayFEM
