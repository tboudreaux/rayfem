#include "../src/app/figure_writers.hpp"

#include <iostream>
#include <regex>
#include <stdexcept>
#include <string>

namespace {
void Check(bool ok, const std::string& what) { if (!ok) throw std::runtime_error(what); }
using namespace RayFEM::UI::Figure;

void Ticks() {
    const auto ticks = NiceTicks(0, 1);
    Check(ticks.size() >= 5 && ticks.front() == 0 && std::abs(ticks.back() - 1) < 1e-12, "Unit range ticks include both ends");
    for (std::size_t i = 1; i < ticks.size(); ++i) Check(std::abs(ticks[i] - ticks[i - 1] - .2) < 1e-12, "Ticks step by 0.2");
    const auto odd = NiceTicks(-3.7, 12.2);
    for (double t : odd) Check(t >= -3.7 && t <= 12.2, "Ticks stay inside the range");
    Check(NiceTicks(1, 1).empty() && NiceTicks(NAN, 2).empty(), "Degenerate ranges have no ticks");
    Check(FormatNumber(.5, .25) == "0.50", "Decimals follow the step");
    Check(FormatNumber(20000, 5000) == "20000", "Integers stay integers");
    Check(LatexNumber("1.5e+06") == "$1.5\\times10^{6}$", "Exponents become LaTeX powers");
}

void Text() {
    Check(PlainFromLatex("$\\varphi$") == "φ", "varphi");
    Check(PlainFromLatex("$30^\\circ$") == "30°", "degrees");
    Check(PlainFromLatex("$r/R_\\odot$").find("r/R") == 0, "Unknown macros keep their letters");
    Check(LatexText("a_b & 50%") == "a\\_b \\& 50\\%", "Text escapes");
    const auto runs = PdfRuns("θ = 30°");
    Check(runs.size() == 2 && runs[0].symbol && runs[0].bytes == "q" && !runs[1].symbol && runs[1].bytes == " = 30\xB0",
          "Greek uses the Symbol font, degrees WinAnsi");
}

// Axes around a unit cube seen from a corner, with the far half hidden.
void Axes() {
    const BoundingBox box{{-1, -1, -1}, {1, 1, 1}};
    Camera3D camera{};
    camera.position = {4, -5, 3}; camera.target = {0, 0, 0}; camera.up = {0, 0, 1}; camera.fovy = 40;
    const View view(camera, {800, 600});
    // A sphere of radius 0.8 inside the box stands in for the mesh: a point
    // is visible unless the sphere is hit first along the view ray.
    const Visibility nearSide = [&](Vector3 p) {
        const Vector3 d = Vector3Subtract(p, camera.position);
        const float length = Vector3Length(d);
        const Vector3 u = Vector3Scale(d, 1/length);
        const float b = Vector3DotProduct(camera.position, u), c = Vector3DotProduct(camera.position, camera.position) - .64f;
        const float disc = b*b - c;
        if (disc < 0) return true;
        const float hit = -b - std::sqrt(disc);
        return hit < 0 || hit >= length - 1e-4f;
    };
    const Measure measure = [](const std::string& text, float px) { return .55f*px*float(text.size()); };
    Ink ink;
    for (auto style : {AxesStyle::Cartesian, AxesStyle::Cylindrical, AxesStyle::Spherical, AxesStyle::Polar}) {
        AxesSettings settings;
        settings.style = style;
        Overlay overlay;
        AddAxes(overlay, settings, box, view, nearSide, ink, measure);
        Check(!overlay.strokes.empty() && !overlay.labels.empty(), "Every axis style draws ticks and labels");
        bool dashed = false;
        for (const auto& s : overlay.strokes) dashed |= s.dashed;
        Check(style == AxesStyle::Cartesian || dashed, "Hidden parts are dashed");
        for (const auto& l : overlay.labels) Check(!l.plain.empty() && !l.latex.empty(), "Labels carry plain and LaTeX text");
        settings.showHidden = false;
        Overlay visibleOnly;
        AddAxes(visibleOnly, settings, box, view, nearSide, ink, measure);
        for (const auto& s : visibleOnly.strokes) Check(!s.dashed, "Hidden parts can be left out");
    }
    // Titles default per style and can be overridden.
    AxesSettings named;
    named.style = AxesStyle::Spherical;
    named.titles = {"$r/R$", "", ""};
    Overlay overlay;
    AddAxes(overlay, named, box, view, {}, ink, measure);
    bool custom = false, theta = false;
    for (const auto& l : overlay.labels) { custom |= l.latex == "$r/R$"; theta |= l.latex == "$\\theta$"; }
    Check(custom && theta, "Custom and default titles");
    // Ring radius: fitted with padding by default, or set; a larger ring
    // reaches further on screen.
    for (auto style : {AxesStyle::Cylindrical, AxesStyle::Spherical, AxesStyle::Polar}) {
        AxesSettings small, large;
        small.style = large.style = style;
        small.radius = .5f; large.radius = 3;
        Overlay a, b;
        AddAxes(a, small, box, view, {}, ink, measure);
        AddAxes(b, large, box, view, {}, ink, measure);
        const Rectangle ea = Extent(a, measure, {400, 300, 0, 0}), eb = Extent(b, measure, {400, 300, 0, 0});
        Check(eb.width > ea.width*1.8f && eb.height > ea.height*1.5f, "A custom ring radius sizes the rings");
    }
    AxesSettings fitted;
    Check(std::abs(fitted.RingRadius(2) - 2.16f) < 1e-5f, "Default rings inscribe the mesh with 8% padding");
    fitted.padding = .25f;
    Check(std::abs(fitted.RingRadius(2) - 2.5f) < 1e-5f, "Padding is a fraction of the mesh radius");
    // Label and title switches, title placement along an axis, angle step.
    {
        AxesSettings cartesian;
        cartesian.style = AxesStyle::Cartesian;
        Ink quiet = ink;
        quiet.labels = false;
        Overlay none;
        AddAxes(none, cartesian, box, view, {}, quiet, measure);
        for (const auto& l : none.labels) Check(l.size == quiet.titleFont, "Hidden tick labels leave only titles");
        quiet.titles = false;
        Overlay bare;
        AddAxes(bare, cartesian, box, view, {}, quiet, measure);
        Check(bare.labels.empty() && !bare.strokes.empty(), "Without labels or titles only the axes remain");
        Ink start = ink, end = ink;
        start.titleAlong = 0; end.titleAlong = 1;
        Overlay first, last;
        AddAxes(first, cartesian, box, view, {}, start, measure);
        AddAxes(last, cartesian, box, view, {}, end, measure);
        auto title = [&](const Overlay& o) { for (const auto& l : o.labels) if (l.latex == "$x$") return l.at; return Vector2{}; };
        Check(Vector2Distance(title(first), title(last)) > 100, "Axis titles move along their axis");
        AxesSettings rings;
        rings.style = AxesStyle::Polar;
        Overlay coarse, fine;
        rings.angleStep = 90; AddAxes(coarse, rings, box, view, {}, ink, measure);
        rings.angleStep = 30; AddAxes(fine, rings, box, view, {}, ink, measure);
        Check(fine.labels.size() > coarse.labels.size(), "A smaller angle step gives more ring labels");
    }
    // A colorbar on the left puts its labels left of the bar.
    {
        Overlay left;
        AddColorBar(left, {100, 100, 10, 200}, RayFEM::ResolveColorScale(0, 1), RayFEM::ColorMap::Viridis, false, "", ink, true);
        for (const auto& l : left.labels) Check(l.at.x < 100 && l.align.x == 1, "Left colorbar labels sit left of it");
    }
    // Overlapping labels: the title stays, the clashing tick label goes.
    Overlay clash;
    clash.labels.push_back({{100, 100}, "0.5", "$0.5$", 12, {.5f, .5f}, BLACK});
    clash.labels.push_back({{104, 102}, "z", "$z$", 14, {.5f, .5f}, BLACK});
    clash.labels.push_back({{300, 100}, "1.0", "$1.0$", 12, {.5f, .5f}, BLACK});
    RemoveOverlaps(clash, measure);
    Check(clash.labels.size() == 2 && clash.labels[0].plain == "z" && clash.labels[1].plain == "1.0", "Overlapping labels are dropped, titles kept");
    // Colorbar: slices, a frame, five ticks and the title.
    Overlay bar;
    AddColorBar(bar, {700, 100, 12, 300}, RayFEM::ResolveColorScale(0, 4), RayFEM::ColorMap::Viridis, false, "density", ink);
    Check(bar.fills.size() == 128 && bar.labels.size() == 6, "Colorbar slices and labels");
    const Rectangle extent = Extent(bar, measure, {0, 0, 800, 600});
    Check(extent.x + extent.width > 712 && extent.y < 100, "Extent covers labels outside the scene");
}

std::size_t Count(const std::string& text, const std::string& what) {
    std::size_t n = 0;
    for (auto at = text.find(what); at != std::string::npos; at = text.find(what, at + 1)) ++n;
    return n;
}

void Pdf(const TextMetrics& metrics) {
    Image scene = GenImageColor(40, 30, Color{10, 120, 200, 255});
    ImageDrawRectangle(&scene, 0, 0, 10, 10, Color{0, 0, 0, 0});  // a transparent corner
    Overlay overlay;
    overlay.strokes.push_back({{{0, 0}, {50, 40}}, BLACK, 1.5f, false});
    overlay.strokes.push_back({{{0, 40}, {50, 0}}, ColorAlpha(BLACK, .5f), 1, true});
    overlay.fills.push_back({{45, 5, 4, 20}, RED});
    overlay.labels.push_back({{25, 35}, "φ (rad)", "$\\varphi$", 9, {.5f, .5f}, BLACK});
    const std::string pdf = BuildPdf({60, 45}, 300, WHITE, scene, {5, 5}, overlay, metrics);
    UnloadImage(scene);
    Check(pdf.starts_with("%PDF-1.4") && pdf.ends_with("%%EOF\n"), "PDF header and trailer");
    Check(pdf.find("/SMask") != std::string::npos, "Transparency becomes a soft mask");
    Check(pdf.find("/ExtGState") != std::string::npos, "Translucent strokes use an opacity state");
    Check(pdf.find("/BaseFont /Symbol") != std::string::npos, "Greek text uses Symbol");
    // Every cross-reference entry must point at the start of its object.
    const auto xref = pdf.rfind("startxref\n");
    const std::size_t table = std::stoul(pdf.substr(xref + 10));
    Check(pdf.compare(table, 4, "xref") == 0, "startxref points at the table");
    std::regex entry("(\\d{10}) 00000 n ");
    std::size_t object = 1;
    for (std::sregex_iterator it(pdf.begin() + table, pdf.end(), entry), end; it != end; ++it, ++object) {
        const std::size_t offset = std::stoul((*it)[1]);
        Check(pdf.compare(offset, std::to_string(object).size() + 6, std::to_string(object) + " 0 obj") == 0, "Object offsets are exact");
    }
    Check(object - 1 == Count(pdf, " 0 obj\n"), "Every object is indexed");
    // Streams are zlib: header 0x78 0x01 after "stream\n".
    for (auto at = pdf.find("stream\n"); at != std::string::npos; at = pdf.find("stream\n", at + 1))
        if (at < 3 || pdf.compare(at - 3, 3, "end") != 0) Check(pdf[at + 7] == '\x78' && pdf[at + 8] == '\x01', "Streams carry a zlib header");
}

// With a TeX installation: a TikZ figure compiles, and dvipng labels match
// their requested size.
void Latex() {
    const auto tools = LatexTools::Discover();
    if (!tools.Any()) { std::cout << "SKIP LaTeX: no engine found\n"; return; }
    const auto directory = TemporaryDirectory();
    Overlay overlay;
    overlay.strokes.push_back({{{10, 10}, {200, 120}}, BLACK, 2, true});
    overlay.fills.push_back({{220, 10, 12, 100}, BLUE});
    overlay.labels.push_back({{100, 130}, "θ", "$\\theta$", 40, {.5f, 0}, BLACK});
    overlay.labels.push_back({{240, 60}, "density_1", "\\text{" + LatexText("density_1") + "}", 40, {0, .5f}, BLACK});
    Image scene = GenImageColor(64, 48, ORANGE);
    ExportImage(scene, (directory/"mesh.png").string().c_str());
    UnloadImage(scene);
    const auto engine = tools.engines.count("pdflatex") ? std::string("pdflatex") : tools.engines.begin()->first;
    std::string tex = BuildTikz({320, 200}, 300, std::nullopt, "mesh.png", {20, 20}, {64, 48}, overlay, engine);
    // Uncompressed objects so the page box can be read back below.
    if (engine == "pdflatex") Replace(tex, "\\begin{document}", "\\pdfobjcompresslevel=0\n\\begin{document}");
    WriteText(directory/"figure.tex", tex);
    const auto error = CompileLatex(tools.engines.at(engine), directory, "figure");
    Check(!error, "TikZ figure compiles with " + engine + ": " + error.value_or(""));
    const std::string pdf = ReadText(directory/"figure.pdf");
    // 320 px at 300 dpi is 76.8 bp wide.
    std::smatch box;
    if (engine == "pdflatex")
        Check(std::regex_search(pdf, box, std::regex("/MediaBox\\s*\\[\\s*0\\s+0\\s+([0-9.]+)\\s+([0-9.]+)")) &&
              std::abs(std::stod(box[1]) - 76.8) < .5 && std::abs(std::stod(box[2]) - 48) < .5, "Page size follows pixels and dpi");
    if (tools.PngLabels()) {
        std::string labelError;
        auto images = RenderLatexLabels(tools, overlay.labels, 300, directory, labelError);
        Check(labelError.empty() && images.size() == 2, "dvipng renders each label: " + labelError);
        // A 40 px theta is roughly 40 px tall ink, never tiny or huge.
        Check(images[0].height > 15 && images[0].height < 60, "Label size follows the requested pixels");
        bool transparent = false;
        const auto* pixels = static_cast<const unsigned char*>(images[0].data);
        for (int i = 0; i < images[0].width*images[0].height; ++i) transparent |= pixels[4*i + 3] == 0;
        Check(transparent, "Labels are transparent around the ink");
        for (auto& image : images) UnloadImage(image);
    }
    std::filesystem::remove_all(directory);
    std::cout << "PASS LaTeX with " << engine << (tools.PngLabels() ? " and dvipng\n" : "\n");
}
} // namespace

int main() {
    try {
        Ticks(); Text(); Axes();
        const TextMetrics metrics;
        Check(metrics.Width("MMMM", 10) > metrics.Width("iiii", 10), "Metrics follow glyph widths");
        Pdf(metrics); Latex();
        std::cout << "Figure checks passed.\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << "Figure test failed: " << error.what() << '\n'; return 1; }
}
