#pragma once

#include "libRayfem/mesh/color_scale.hpp"
#include "libRayfem/mesh/colormap.hpp"
#include "raylib.h"
#include "raymath.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

// A figure is the rendered scene plus a vector overlay (axes, colorbar and
// labels) in final-image pixels, origin at the top left. The overlay is the
// same for every output: rasterized for PNG, written as PDF paths, or as TikZ.
namespace RayFEM::UI::Figure {

enum class AxesStyle { None, Cartesian, Cylindrical, Spherical, Polar };
enum class Background { Theme, White, Transparent };

// align is the anchor's place in the label box: 0, .5 or 1 of its width
// (left, center, right) and height (top, middle, bottom).
struct Label { Vector2 at{}; std::string plain, latex; float size = 12; Vector2 align{.5f, .5f}; Color color = BLACK; };
struct Stroke { std::vector<Vector2> points; Color color = BLACK; float width = 1; bool dashed = false; };
struct Fill { Rectangle box{}; Color color = BLACK; };
struct Overlay {
    std::vector<Stroke> strokes;
    std::vector<Fill> fills;
    std::vector<Label> labels;
    void Offset(Vector2 by) {
        for (auto& s : strokes) for (auto& p : s.points) p = Vector2Add(p, by);
        for (auto& f : fills) { f.box.x += by.x; f.box.y += by.y; }
        for (auto& l : labels) l.at = Vector2Add(l.at, by);
    }
};

// Ink: colors and sizes in final-image pixels.
struct Ink {
    Color color = BLACK, hidden = GRAY;
    float font = 12, line = 1.5f, tick = 6;
    float labelGap = 4;                // tick end to its label
    float titleFont = 13, titleGap = 8;  // titles sit this far beyond the labels
    float titleAlong = .5f;            // a ruler title's place, 0 at its start to 1 at its end
    bool labels = true, titles = true;
    int ticks = 5;                     // about this many ticks per ruler
};

struct AxesSettings {
    AxesStyle style = AxesStyle::None;
    Vector3 origin{};
    Vector3 axis{0, 0, 1};
    std::array<std::string, 3> titles{};  // LaTeX; empty uses the style's default
    bool showHidden = true;                // dashed where the mesh covers an axis
    // Rings (cylindrical, spherical, polar): a set radius, or 0 to fit the
    // mesh and add padding as a fraction of its radius.
    float radius = 0, padding = .08f;
    int angleStep = 30;                    // degrees between ring ticks
    float RingRadius(float meshRadius) const { return radius > 0 ? radius : meshRadius*(1 + padding); }
};

// Width of a plain UTF-8 label at a pixel size (for layout).
using Measure = std::function<float(const std::string&, float)>;
// Whether a point is in front of the rendered mesh.
using Visibility = std::function<bool(Vector3)>;

// Projects with the scene camera onto an image of the given size.
struct View {
    Camera3D camera{};
    Vector2 size{};
    Matrix viewProjection{};
    View(const Camera3D& c, Vector2 s) : camera(c), size(s) {
        viewProjection = MatrixMultiply(MatrixLookAt(c.position, c.target, c.up),
                                        MatrixPerspective(c.fovy*DEG2RAD, double(s.x)/double(s.y), .01, 1000));
    }
    std::optional<Vector2> Project(Vector3 p) const {
        const Quaternion q = QuaternionTransform({p.x, p.y, p.z, 1}, viewProjection);
        if (!(q.w > 1e-6f)) return std::nullopt;
        return Vector2{(q.x/q.w + 1)*.5f*size.x, (1 - q.y/q.w)*.5f*size.y};
    }
    Vector3 Forward() const { return Vector3Normalize(Vector3Subtract(camera.target, camera.position)); }
    // A point far along the view ray through a screen position.
    Vector3 Behind(Vector2 screen) const {
        const Ray ray = GetScreenToWorldRayEx(screen, camera, int(size.x), int(size.y));
        const float reach = 1e4f*std::max(1.f, Vector3Distance(camera.position, camera.target));
        return Vector3Add(ray.position, Vector3Scale(ray.direction, reach));
    }
};
// Whether the mesh is drawn at a screen position (labels there would sit on it).
inline bool Covered(const View& view, const Visibility& visible, Vector2 screen) {
    return visible && !visible(view.Behind(screen));
}

// ---- Text --------------------------------------------------------------

inline void Replace(std::string& text, const std::string& from, const std::string& to) {
    for (std::size_t at = text.find(from); at != std::string::npos; at = text.find(from, at + to.size()))
        text.replace(at, from.size(), to);
}
// A readable plain form of a short LaTeX label: math markers and grouping
// go, common symbols become Unicode.
inline std::string PlainFromLatex(std::string text) {
    for (const auto& [from, to] : std::initializer_list<std::pair<const char*, const char*>>{
             {"^\\circ", "°"}, {"^{\\circ}", "°"}, {"\\circ", "°"}, {"\\varphi", "φ"}, {"\\phi", "φ"},
             {"\\theta", "θ"}, {"\\rho", "ρ"}, {"\\pi", "π"}, {"\\times", "×"}, {"\\cdot", "·"},
             {"\\,", " "}, {"\\ ", " "}, {"\\mathrm", ""}, {"\\text", ""}, {"\\left", ""}, {"\\right", ""}})
        Replace(text, from, to);
    std::string out;
    for (char c : text) if (c != '$' && c != '{' && c != '}' && c != '\\') out += c;
    Replace(out, "-", "−");
    return out;
}
// Escape plain text for LaTeX text mode.
inline std::string LatexText(const std::string& text) {
    std::string out;
    for (char c : text) {
        switch (c) {
            case '\\': out += "\\textbackslash{}"; break;
            case '&': case '%': case '$': case '#': case '_': case '{': case '}': out += '\\'; out += c; break;
            case '~': out += "\\textasciitilde{}"; break;
            case '^': out += "\\textasciicircum{}"; break;
            default: out += c;
        }
    }
    return out;
}

// ---- Ticks ---------------------------------------------------------------

// About target values at 1, 2 or 5 times a power of ten, inside [low, high].
inline std::vector<double> NiceTicks(double low, double high, int target = 5) {
    std::vector<double> ticks;
    if (!std::isfinite(low) || !std::isfinite(high) || !(high > low)) return ticks;
    const double raw = (high - low)/std::max(1, target);
    const double power = std::pow(10.0, std::floor(std::log10(raw)));
    double step = power;
    for (double m : {1.0, 2.0, 5.0, 10.0}) if (raw <= m*power*1.0000001) { step = m*power; break; }
    const double first = std::ceil(low/step - 1e-9)*step;
    for (double v = first; v <= high + step*1e-9 && ticks.size() < 64; v += step)
        ticks.push_back(std::abs(v) < step*1e-9 ? 0.0 : v);
    return ticks;
}
inline double TickStep(const std::vector<double>& ticks) { return ticks.size() > 1 ? ticks[1] - ticks[0] : 1; }
// Fewest digits that keep neighbouring ticks distinct.
inline std::string FormatNumber(double value, double step) {
    char text[64];
    const double magnitude = std::max(std::abs(value), std::abs(step));
    if (magnitude != 0 && (magnitude >= 1e5 || magnitude < 1e-3)) std::snprintf(text, sizeof(text), "%.3g", value);
    else {
        // As many decimals as the step needs to be exact (0.25 needs two).
        int decimals = 0;
        for (double scaled = std::abs(step); decimals < 8 && std::abs(scaled - std::round(scaled)) > 1e-6*std::max(1.0, scaled); scaled *= 10)
            ++decimals;
        std::snprintf(text, sizeof(text), "%.*f", decimals, value);
    }
    return text;
}
inline std::string LatexNumber(const std::string& plain) {
    std::string out = plain;
    const auto e = out.find('e');
    if (e != std::string::npos) {
        const int exponent = std::stoi(out.substr(e + 1));
        out = out.substr(0, e) + "\\times10^{" + std::to_string(exponent) + "}";
    }
    return "$" + out + "$";
}
inline Label NumberLabel(double value, double step, Vector2 at, Vector2 align, const Ink& ink) {
    const std::string plain = FormatNumber(value, step);
    std::string shown = plain;
    Replace(shown, "-", "−");
    return {at, shown, LatexNumber(plain), ink.font, align, ink.color};
}
inline Label AngleLabel(int degrees, Vector2 at, Vector2 align, const Ink& ink) {
    return {at, std::to_string(degrees) + "°", "$" + std::to_string(degrees) + "^\\circ$", ink.font, align, ink.color};
}

// ---- Curves with hidden-line handling -----------------------------------

// Split a sampled 3D curve into runs in front of and behind the mesh.
inline void AddCurve(Overlay& overlay, const std::vector<Vector3>& points, const View& view,
                     const Visibility& visible, const Ink& ink, bool showHidden, float width) {
    Stroke run;
    bool runVisible = true;
    auto flush = [&] {
        if (run.points.size() > 1 && (runVisible || showHidden)) {
            run.dashed = !runVisible;
            run.color = runVisible ? ink.color : ink.hidden;
            run.width = runVisible ? width : width*.8f;
            overlay.strokes.push_back(run);
        }
        run.points.clear();
    };
    for (const auto& p : points) {
        const auto screen = view.Project(p);
        if (!screen) { flush(); continue; }
        const bool front = !visible || visible(p);
        if (!run.points.empty() && front != runVisible) {
            const Vector2 last = run.points.back();
            flush();
            run.points.push_back(last);
        }
        runVisible = front;
        run.points.push_back(*screen);
    }
    flush();
}
inline std::vector<Vector3> Segment(Vector3 a, Vector3 b, int samples = 96) {
    std::vector<Vector3> points;
    for (int i = 0; i <= samples; ++i) points.push_back(Vector3Lerp(a, b, float(i)/samples));
    return points;
}
inline std::vector<Vector3> Arc(Vector3 center, Vector3 u, Vector3 v, float radius, double from, double to, int samples = 180) {
    std::vector<Vector3> points;
    for (int i = 0; i <= samples; ++i) {
        const double t = from + (to - from)*i/samples;
        points.push_back(Vector3Add(center, Vector3Add(Vector3Scale(u, radius*float(std::cos(t))), Vector3Scale(v, radius*float(std::sin(t))))));
    }
    return points;
}

// Snap an outward direction to one of nine label anchors.
inline Vector2 AlignFor(Vector2 outward) {
    auto snap = [](float c) { return c > .38f ? 0.f : c < -.38f ? 1.f : .5f; };
    return {snap(outward.x), snap(outward.y)};
}
inline Vector2 Normalized(Vector2 v) { const float l = Vector2Length(v); return l > 1e-6f ? Vector2Scale(v, 1/l) : Vector2{0, 1}; }

// Tick marks and labels along a straight 3D ruler, pointing outward on screen.
inline void AddRuler(Overlay& overlay, Vector3 from, Vector3 to, double low, double high, Vector2 outward,
                     const View& view, const Visibility& visible, const Ink& ink, bool showHidden,
                     const std::string& title, const Measure& measure) {
    AddCurve(overlay, Segment(from, to), view, visible, ink, showHidden, ink.line);
    const auto ticks = NiceTicks(low, high, ink.ticks);
    const double step = TickStep(ticks);
    const Vector2 align = AlignFor(outward);
    float labelReach = 0;
    for (double value : ticks) {
        const Vector3 p = Vector3Lerp(from, to, float((value - low)/(high - low)));
        const auto screen = view.Project(p);
        if (!screen) continue;
        const bool front = !visible || visible(p);
        if (!front && !showHidden) continue;
        overlay.strokes.push_back({{*screen, Vector2Add(*screen, Vector2Scale(outward, ink.tick))}, front ? ink.color : ink.hidden, ink.line*.8f});
        // A label over the mesh would clutter it; its tick remains.
        const Vector2 at = Vector2Add(*screen, Vector2Scale(outward, ink.tick + ink.labelGap));
        if (!ink.labels || Covered(view, visible, Vector2Add(at, Vector2Scale(outward, ink.font*.5f)))) continue;
        auto label = NumberLabel(value, step, at, align, ink);
        labelReach = std::max(labelReach, std::abs(outward.x)*measure(label.plain, ink.font) + std::abs(outward.y)*ink.font);
        overlay.labels.push_back(label);
    }
    if (title.empty() || !ink.titles) return;
    const auto a = view.Project(from), b = view.Project(to);
    if (!a || !b) return;
    const Vector2 place = Vector2Lerp(*a, *b, std::clamp(ink.titleAlong, 0.f, 1.f));
    const Vector2 at = Vector2Add(place, Vector2Scale(outward, ink.tick + ink.labelGap + labelReach + ink.titleGap));
    if (Covered(view, visible, at)) return;  // a ruler hidden by the mesh keeps its title off it too
    overlay.labels.push_back({at, PlainFromLatex(title), title, ink.titleFont, align, ink.color});
}

// Angle ticks every step degrees around a ring, labels outside it.
inline void AddAngleRing(Overlay& overlay, Vector3 center, Vector3 u, Vector3 v, float radius, int fromDegrees, int toDegrees,
                         int step, const View& view, const Visibility& visible, const Ink& ink, bool showHidden, int unlabeled = -1) {
    const auto c = view.Project(center);
    AddCurve(overlay, Arc(center, u, v, radius, fromDegrees*DEG2RAD, toDegrees*DEG2RAD), view, visible, ink, showHidden, ink.line);
    const bool full = toDegrees - fromDegrees >= 360;
    step = std::max(1, step);
    for (int degrees = fromDegrees; degrees <= toDegrees - (full ? step : 0); degrees += step) {
        const double t = degrees*DEG2RAD;
        const Vector3 direction = Vector3Add(Vector3Scale(u, float(std::cos(t))), Vector3Scale(v, float(std::sin(t))));
        const Vector3 p = Vector3Add(center, Vector3Scale(direction, radius));
        const auto screen = view.Project(p);
        if (!screen || !c) continue;
        const bool front = !visible || visible(p);
        if (!front && !showHidden) continue;
        const Vector2 outward = Normalized(Vector2Subtract(*screen, *c));
        overlay.strokes.push_back({{*screen, Vector2Add(*screen, Vector2Scale(outward, ink.tick))}, front ? ink.color : ink.hidden, ink.line*.8f});
        const Vector2 at = Vector2Add(*screen, Vector2Scale(outward, ink.tick + ink.labelGap));
        if (!ink.labels || degrees == unlabeled || Covered(view, visible, Vector2Add(at, Vector2Scale(outward, ink.font*.5f)))) continue;
        overlay.labels.push_back(AngleLabel(degrees, at, AlignFor(outward), ink));
    }
}

// A title just outside a ring at the ring point lowest on screen.
inline void AddRingTitle(Overlay& overlay, Vector3 center, Vector3 u, Vector3 v, float radius, const View& view,
                         const std::string& title, const Ink& ink) {
    if (title.empty() || !ink.titles) return;
    const float clearance = ink.tick + (ink.labels ? ink.labelGap + ink.font*1.2f : 0) + ink.titleGap;
    std::optional<Vector2> lowest;
    for (int i = 0; i < 72; ++i) {
        const double t = i*5*DEG2RAD;
        const auto s = view.Project(Vector3Add(center, Vector3Add(Vector3Scale(u, radius*float(std::cos(t))), Vector3Scale(v, radius*float(std::sin(t))))));
        if (s && (!lowest || s->y > lowest->y)) lowest = s;
    }
    if (!lowest) return;
    overlay.labels.push_back({{lowest->x, lowest->y + clearance}, PlainFromLatex(title), title, ink.titleFont, {.5f, 0}, ink.color});
}

inline std::array<std::string, 3> DefaultTitles(AxesStyle style) {
    switch (style) {
        case AxesStyle::Cartesian: return {"$x$", "$y$", "$z$"};
        case AxesStyle::Cylindrical: return {"$r$", "$\\varphi$", "$z$"};
        case AxesStyle::Spherical: return {"$r$", "$\\theta$", "$\\varphi$"};
        case AxesStyle::Polar: return {"$r$", "$\\theta$", ""};
        default: return {};
    }
}

// Unit axis plus two perpendicular in-plane directions.
struct Frame { Vector3 a, e, f; };
inline Frame MakeFrame(Vector3 axis) {
    Frame frame;
    frame.a = Vector3Length(axis) > 0 ? Vector3Normalize(axis) : Vector3{0, 0, 1};
    const Vector3 helper = std::abs(frame.a.x) < .9f ? Vector3{1, 0, 0} : Vector3{0, 1, 0};
    frame.e = Vector3Normalize(Vector3Subtract(helper, Vector3Scale(frame.a, Vector3DotProduct(helper, frame.a))));
    frame.f = Vector3CrossProduct(frame.a, frame.e);
    return frame;
}
// Half of the bounds' extent along a unit direction.
inline float HalfExtent(BoundingBox b, Vector3 d) {
    const Vector3 size = Vector3Subtract(b.max, b.min);
    return .5f*(std::abs(size.x*d.x) + std::abs(size.y*d.y) + std::abs(size.z*d.z));
}
// The in-plane direction (of e, f) that appears furthest to screen-left.
inline Vector3 LeftmostDirection(const Frame& frame, Vector3 center, float radius, const View& view) {
    Vector3 best = frame.e;
    float bestX = INFINITY;
    for (int i = 0; i < 72; ++i) {
        const double t = i*5*DEG2RAD;
        const Vector3 d = Vector3Add(Vector3Scale(frame.e, float(std::cos(t))), Vector3Scale(frame.f, float(std::sin(t))));
        if (const auto s = view.Project(Vector3Add(center, Vector3Scale(d, radius))); s && s->x < bestX) { bestX = s->x; best = d; }
    }
    return best;
}

// Cartesian axes on box edges, in the manner of matplotlib. The axes across
// the screen stay on the floor (the face of the upright axis lower on
// screen); of the two floor edges each takes the one whose labels the mesh
// leaves clear, so a blocked axis jumps to the other side rather than
// climbing over the mesh. The upright axis chooses among the four vertical
// edges the same way, preferring the left. Ties go to the box's outline.
inline void AddCartesianAxes(Overlay& overlay, const AxesSettings& settings, BoundingBox bounds, const View& view,
                             const Visibility& visible, const Ink& ink, const Measure& measure,
                             const std::array<std::string, 3>& titles) {
    const Vector3 center = Vector3Scale(Vector3Add(bounds.min, bounds.max), .5f);
    const auto c = view.Project(center);
    if (!c) return;
    const float low[3] = {bounds.min.x, bounds.min.y, bounds.min.z}, high[3] = {bounds.max.x, bounds.max.y, bounds.max.z};
    auto point = [](const float (&p)[3]) { return Vector3{p[0], p[1], p[2]}; };
    auto faceCenter = [&](int axis, int side) {
        float p[3] = {center.x, center.y, center.z};
        p[axis] = side ? high[axis] : low[axis];
        return point(p);
    };
    auto facing = [&](int axis, int side) {
        Vector3 normal{};
        (&normal.x)[axis] = side ? 1.f : -1.f;
        return Vector3DotProduct(normal, Vector3Subtract(view.camera.position, faceCenter(axis, side))) > 0;
    };
    // The upright axis runs most nearly up the screen; its floor side is the
    // face lower on screen.
    int up = 2;
    float steepest = -1;
    for (int axis = 0; axis < 3; ++axis) {
        const auto e = view.Project(faceCenter(axis, 1));
        if (!e) continue;
        const Vector2 d = Vector2Subtract(*e, *c);
        const float length = Vector2Length(d);
        if (length > 1 && std::abs(d.y)/length > steepest) { steepest = std::abs(d.y)/length; up = axis; }
    }
    int floor = 0;
    if (const auto a = view.Project(faceCenter(up, 0)), b = view.Project(faceCenter(up, 1)); a && b) floor = b->y > a->y ? 1 : 0;
    const float scale = std::max(view.size.x, view.size.y);
    for (int axis = 0; axis < 3; ++axis) {
        if (!(high[axis] > low[axis])) continue;
        const int b = (axis + 1)%3, d = (axis + 2)%3;
        const auto ticks = NiceTicks(low[axis], high[axis], ink.ticks);
        float bestScore = -INFINITY;
        Vector3 from{}, to{};
        Vector2 outward{};
        for (int corner = 0; corner < 4; ++corner) {
            const int sideB = corner & 1, sideD = (corner >> 1) & 1;
            if (axis != up && (b == up ? sideB : sideD) != floor) continue;  // stay on the floor
            float p[3], q[3];
            p[axis] = low[axis]; q[axis] = high[axis];
            p[b] = q[b] = sideB ? high[b] : low[b];
            p[d] = q[d] = sideD ? high[d] : low[d];
            const auto s0 = view.Project(point(p)), s1 = view.Project(point(q));
            const auto m = view.Project(Vector3Scale(Vector3Add(point(p), point(q)), .5f));
            if (!s0 || !s1 || !m || Vector2Distance(*s0, *s1) < ink.font*2) continue;
            const Vector2 along = Normalized(Vector2Subtract(*s1, *s0));
            Vector2 out = Vector2Subtract(*m, *c);
            out = Vector2Subtract(out, Vector2Scale(along, Vector2DotProduct(out, along)));
            out = Vector2Length(out) > 1 ? Normalized(out) : Vector2{-along.y, along.x};
            // Labels the mesh leaves clear decide; then the outline, then
            // the outer (or, upright, the left) side.
            int clear = 0;
            for (double value : ticks) {
                const auto t = view.Project(Vector3Lerp(point(p), point(q), float((value - low[axis])/(high[axis] - low[axis]))));
                if (t && !Covered(view, visible, Vector2Add(*t, Vector2Scale(out, ink.tick + ink.labelGap + ink.font*.5f)))) ++clear;
            }
            const bool outline = facing(b, sideB) != facing(d, sideD);
            const float side = axis == up ? -(m->x - c->x)/scale : Vector2Distance(*m, *c)/scale;
            const float score = clear*10.f + (outline ? 2.f : 0.f) + side;
            if (score > bestScore) { bestScore = score; from = point(p); to = point(q); outward = out; }
        }
        if (!(bestScore > -INFINITY)) continue;  // seen end-on
        AddRuler(overlay, from, to, low[axis], high[axis], outward, view, visible, ink, settings.showHidden, titles[axis], measure);
    }
}

inline void AddCylindricalAxes(Overlay& overlay, const AxesSettings& settings, BoundingBox bounds, const View& view,
                               const Visibility& visible, const Ink& ink, const Measure& measure,
                               const std::array<std::string, 3>& titles) {
    const Frame frame = MakeFrame(settings.axis);
    const Vector3 o = settings.origin;
    float zLow = INFINITY, zHigh = -INFINITY;
    for (int corner = 0; corner < 8; ++corner) {
        const Vector3 p{corner & 1 ? bounds.max.x : bounds.min.x, corner & 2 ? bounds.max.y : bounds.min.y, corner & 4 ? bounds.max.z : bounds.min.z};
        const float z = Vector3DotProduct(Vector3Subtract(p, o), frame.a);
        zLow = std::min(zLow, z); zHigh = std::max(zHigh, z);
    }
    const float meshRadius = std::max(HalfExtent(bounds, frame.e), HalfExtent(bounds, frame.f));
    if (!(meshRadius > 0)) return;
    const float ring = settings.RingRadius(meshRadius);
    if (!(ring > 0)) return;
    const Vector3 base = Vector3Add(o, Vector3Scale(frame.a, zLow));
    // Azimuth ring around the base, with the radial ruler out to it along phi = 0.
    AddAngleRing(overlay, base, frame.e, frame.f, ring, 0, 360, settings.angleStep, view, visible, ink, settings.showHidden);
    if (const auto start = view.Project(base), end = view.Project(Vector3Add(base, Vector3Scale(frame.e, ring))); start && end) {
        const Vector2 along = Normalized(Vector2Subtract(*end, *start));
        Vector2 out{along.y, -along.x};
        if (out.y < 0) out = Vector2Negate(out);
        AddRuler(overlay, base, Vector3Add(base, Vector3Scale(frame.e, ring)), 0, ring, out, view, visible, ink,
                 settings.showHidden, titles[0], measure);
    }
    AddRingTitle(overlay, base, frame.e, frame.f, ring, view, titles[1], ink);
    // Height ruler on the side that faces screen-left.
    if (zHigh > zLow) {
        const Vector3 side = Vector3Scale(LeftmostDirection(frame, o, ring, view), ring);
        const Vector3 from = Vector3Add(base, side), to = Vector3Add(Vector3Add(o, Vector3Scale(frame.a, zHigh)), side);
        if (const auto a = view.Project(from), b = view.Project(to); a && b) {
            const Vector2 along = Normalized(Vector2Subtract(*b, *a));
            Vector2 out{-along.y, along.x};
            if (out.x > 0) out = Vector2Negate(out);
            AddRuler(overlay, from, to, zLow, zHigh, out, view, visible, ink, settings.showHidden, titles[2], measure);
        }
    }
}

inline void AddSphericalAxes(Overlay& overlay, const AxesSettings& settings, BoundingBox bounds, const View& view,
                             const Visibility& visible, const Ink& ink, const Measure& measure,
                             const std::array<std::string, 3>& titles) {
    const Frame frame = MakeFrame(settings.axis);
    const Vector3 o = settings.origin;
    const float meshRadius = std::max({HalfExtent(bounds, frame.a), HalfExtent(bounds, frame.e), HalfExtent(bounds, frame.f)});
    if (!(meshRadius > 0)) return;
    const float ring = settings.RingRadius(meshRadius);
    if (!(ring > 0)) return;
    // Polar axis through both poles, a little beyond the rings.
    AddCurve(overlay, Segment(Vector3Subtract(o, Vector3Scale(frame.a, ring*1.15f)), Vector3Add(o, Vector3Scale(frame.a, ring*1.15f))),
             view, visible, ink, settings.showHidden, ink.line);
    // Equator with azimuth ticks.
    AddAngleRing(overlay, o, frame.e, frame.f, ring, 0, 360, settings.angleStep, view, visible, ink, settings.showHidden);
    AddRingTitle(overlay, o, frame.e, frame.f, ring, view, titles[2], ink);
    // The meridian facing the camera, from the north pole (theta = 0) round
    // the screen-left side, so it frames the body instead of crossing it.
    Vector3 w = Vector3CrossProduct(view.Forward(), frame.a);
    if (Vector3Length(w) < 1e-4f) w = frame.e;
    w = Vector3Normalize(w);
    if (const auto s = view.Project(Vector3Add(o, w)), c = view.Project(o); s && c && s->x > c->x) w = Vector3Negate(w);
    AddAngleRing(overlay, o, frame.a, w, ring, 0, 180, settings.angleStep, view, visible, ink, settings.showHidden, 90);
    if (const auto top = view.Project(Vector3Add(o, Vector3Scale(Vector3Add(Vector3Scale(frame.a, .7071f), Vector3Scale(w, .7071f)), ring))),
                   c = view.Project(o); top && c && ink.titles && !titles[1].empty()) {
        const Vector2 out = Normalized(Vector2Subtract(*top, *c));
        const float clearance = ink.tick + (ink.labels ? ink.labelGap + ink.font*1.6f : 0) + ink.titleGap;
        overlay.labels.push_back({Vector2Add(*top, Vector2Scale(out, clearance)), PlainFromLatex(titles[1]), titles[1],
                                  ink.titleFont, AlignFor(out), ink.color});
    }
    // Radial ruler out to the rings along the meridian plane's equatorial direction.
    if (const auto start = view.Project(o), end = view.Project(Vector3Add(o, Vector3Scale(w, ring))); start && end) {
        const Vector2 along = Normalized(Vector2Subtract(*end, *start));
        Vector2 out{along.y, -along.x};
        if (out.y < 0) out = Vector2Negate(out);
        AddRuler(overlay, o, Vector3Add(o, Vector3Scale(w, ring)), 0, ring, out, view, visible, ink, settings.showHidden, titles[0], measure);
    }
}

inline void AddPolarAxes(Overlay& overlay, const AxesSettings& settings, BoundingBox bounds, const View& view,
                         const Visibility& visible, const Ink& ink, const Measure& measure,
                         const std::array<std::string, 3>& titles) {
    const Frame frame = MakeFrame(settings.axis);
    const Vector3 o = settings.origin;
    const float meshRadius = std::max(HalfExtent(bounds, frame.e), HalfExtent(bounds, frame.f));
    if (!(meshRadius > 0)) return;
    const float outer = settings.RingRadius(meshRadius);
    if (!(outer > 0)) return;
    const auto ticks = NiceTicks(0, outer, ink.ticks);
    Ink grid = ink;
    grid.color = ColorAlpha(ink.color, .35f);
    grid.hidden = ColorAlpha(ink.hidden, .35f);
    for (double r : ticks) if (r > 0) AddCurve(overlay, Arc(o, frame.e, frame.f, float(r), 0, 2*std::numbers::pi), view, visible, grid, settings.showHidden, ink.line*.7f);
    for (int degrees = 0; degrees < 360; degrees += std::max(1, settings.angleStep)) {
        const double t = degrees*DEG2RAD;
        const Vector3 d = Vector3Add(Vector3Scale(frame.e, float(std::cos(t))), Vector3Scale(frame.f, float(std::sin(t))));
        AddCurve(overlay, Segment(o, Vector3Add(o, Vector3Scale(d, outer)), 32), view, visible, grid, settings.showHidden, ink.line*.7f);
    }
    AddAngleRing(overlay, o, frame.e, frame.f, outer, 0, 360, settings.angleStep, view, visible, ink, settings.showHidden);
    AddRingTitle(overlay, o, frame.e, frame.f, outer, view, titles[1], ink);
    if (const auto start = view.Project(o), end = view.Project(Vector3Add(o, Vector3Scale(frame.e, outer))); start && end) {
        const Vector2 along = Normalized(Vector2Subtract(*end, *start));
        Vector2 out{along.y, -along.x};
        if (out.y < 0) out = Vector2Negate(out);
        AddRuler(overlay, o, Vector3Add(o, Vector3Scale(frame.e, outer)), 0, outer, out, view, visible, ink, settings.showHidden, titles[0], measure);
    }
}

inline void AddAxes(Overlay& overlay, const AxesSettings& settings, BoundingBox bounds, const View& view,
                    const Visibility& visible, const Ink& ink, const Measure& measure) {
    auto titles = DefaultTitles(settings.style);
    for (int i = 0; i < 3; ++i) if (!settings.titles[i].empty()) titles[i] = settings.titles[i];
    switch (settings.style) {
        case AxesStyle::Cartesian: AddCartesianAxes(overlay, settings, bounds, view, visible, ink, measure, titles); break;
        case AxesStyle::Cylindrical: AddCylindricalAxes(overlay, settings, bounds, view, visible, ink, measure, titles); break;
        case AxesStyle::Spherical: AddSphericalAxes(overlay, settings, bounds, view, visible, ink, measure, titles); break;
        case AxesStyle::Polar: AddPolarAxes(overlay, settings, bounds, view, visible, ink, measure, titles); break;
        case AxesStyle::None: break;
    }
}

// A vertical colorbar with ticks along the colorbar's mapped scale.
// Ticks and labels go on the right, or on the left for a bar left of the mesh.
inline void AddColorBar(Overlay& overlay, Rectangle box, const ColorScaleMapping& mapping, ColorMap map, bool reverse,
                        const std::string& title, const Ink& ink, bool labelsLeft = false) {
    constexpr int slices = 128;
    for (int i = 0; i < slices; ++i) {
        const float t = 1 - (i + .5f)/slices;
        const float y0 = box.y + box.height*i/slices, y1 = box.y + box.height*(i + 1)/slices;
        overlay.fills.push_back({{box.x, y0, box.width, y1 - y0 + .35f}, SampleColorMap(map, t, reverse)});
    }
    overlay.strokes.push_back({{{box.x, box.y}, {box.x + box.width, box.y}, {box.x + box.width, box.y + box.height},
                                {box.x, box.y + box.height}, {box.x, box.y}}, ink.color, ink.line*.8f});
    // Five values evenly spaced along the bar, as on screen.
    std::vector<double> values;
    for (int i = 0; i <= 4; ++i) values.push_back(ColorScaleValue(i/4.0, mapping));
    for (int i = 0; i <= 4; ++i) {
        const float y = box.y + box.height*(1 - i/4.f);
        const float edge = labelsLeft ? box.x : box.x + box.width, sign = labelsLeft ? -1.f : 1.f;
        overlay.strokes.push_back({{{edge, y}, {edge + sign*ink.tick, y}}, ink.color, ink.line*.8f});
        char plain[64];
        std::snprintf(plain, sizeof(plain), "%.4g", values[i]);
        std::string shown = plain;
        Replace(shown, "-", "−");
        if (ink.labels)
            overlay.labels.push_back({{edge + sign*(ink.tick + ink.labelGap), y}, shown, LatexNumber(plain), ink.font, {labelsLeft ? 1.f : 0.f, .5f}, ink.color});
    }
    if (!title.empty())
        overlay.labels.push_back({{box.x + box.width*.5f, box.y - ink.font*.6f - ink.labelGap}, title, "\\text{" + LatexText(title) + "}", ink.font, {.5f, 1}, ink.color});
}

// Drop labels that would overlap one already placed, where two scales
// meet. Titles (the larger labels) are placed first so they always stay.
inline void RemoveOverlaps(Overlay& overlay, const Measure& measure) {
    std::vector<std::size_t> order(overlay.labels.size());
    for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::stable_sort(order.begin(), order.end(), [&](auto a, auto b) { return overlay.labels[a].size > overlay.labels[b].size; });
    std::vector<Rectangle> placed;
    std::vector<bool> keep(overlay.labels.size(), false);
    for (auto i : order) {
        const auto& l = overlay.labels[i];
        const float w = measure(l.plain, l.size), h = l.size, gap = l.size*.15f;
        const Rectangle box{l.at.x - l.align.x*w - gap, l.at.y - l.align.y*h - gap, w + 2*gap, h + 2*gap};
        if (std::any_of(placed.begin(), placed.end(), [&](const Rectangle& r) { return CheckCollisionRecs(r, box); })) continue;
        placed.push_back(box);
        keep[i] = true;
    }
    std::vector<Label> kept;
    for (std::size_t i = 0; i < overlay.labels.size(); ++i) if (keep[i]) kept.push_back(std::move(overlay.labels[i]));
    overlay.labels = std::move(kept);
}

// Bounds of everything in the overlay, with label boxes from measure.
inline Rectangle Extent(const Overlay& overlay, const Measure& measure, Rectangle start) {
    float x0 = start.x, y0 = start.y, x1 = start.x + start.width, y1 = start.y + start.height;
    auto grow = [&](float x, float y) { x0 = std::min(x0, x); y0 = std::min(y0, y); x1 = std::max(x1, x); y1 = std::max(y1, y); };
    for (const auto& s : overlay.strokes) for (const auto& p : s.points) grow(p.x - s.width, p.y - s.width), grow(p.x + s.width, p.y + s.width);
    for (const auto& f : overlay.fills) grow(f.box.x, f.box.y), grow(f.box.x + f.box.width, f.box.y + f.box.height);
    for (const auto& l : overlay.labels) {
        const float w = measure(l.plain, l.size)*1.1f, h = l.size*1.3f;
        const float left = l.at.x - l.align.x*w, top = l.at.y - l.align.y*h;
        grow(left, top); grow(left + w, top + h);
    }
    return {x0, y0, x1 - x0, y1 - y0};
}

} // namespace RayFEM::UI::Figure
