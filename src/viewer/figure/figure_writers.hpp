#pragma once

#include "figure/figure.hpp"

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <random>
#include <sstream>
#include <unordered_map>

namespace RayFEM::UI::Figure {

// Glyphs labels may use: printable ASCII and the symbols axis labels need.
inline const std::vector<int>& FigureCodepoints() {
    static const std::vector<int> codepoints = [] {
        std::vector<int> list;
        for (int c = 32; c < 127; ++c) list.push_back(c);
        for (int c : {0xB0, 0xB7, 0xD7, 0x2212, 0x03B8, 0x03C6, 0x03D5, 0x03C1, 0x03C0}) list.push_back(c);
        return list;
    }();
    return codepoints;
}
// The interface's sans-serif font. Arial shares Helvetica's metrics, so the
// PDF writer can set labels in Helvetica with these widths.
inline std::optional<std::string> FigureFontPath() {
    for (const char* path : {"/System/Library/Fonts/Supplemental/Arial.ttf", "/Library/Fonts/Arial.ttf",
                             "/usr/share/fonts/truetype/msttcorefonts/Arial.ttf",
                             "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
                             "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", "C:/Windows/Fonts/arial.ttf"})
        if (std::filesystem::exists(path)) return std::string(path);
    return std::nullopt;
}

// Label widths from glyph advances only (no GPU), usable on any thread.
class TextMetrics {
public:
    TextMetrics() {
        const auto path = FigureFontPath();
        if (!path) return;
        int size = 0;
        unsigned char* data = LoadFileData(path->c_str(), &size);
        if (!data) return;
        auto codepoints = FigureCodepoints();
        GlyphInfo* glyphs = LoadFontData(data, size, base, codepoints.data(), int(codepoints.size()), FONT_DEFAULT);
        if (glyphs) {
            for (std::size_t i = 0; i < codepoints.size(); ++i) advances[glyphs[i].value] = float(glyphs[i].advanceX);
            UnloadFontData(glyphs, int(codepoints.size()));
        }
        UnloadFileData(data);
    }
    float Width(const std::string& text, float px) const {
        float width = 0;
        for (std::size_t i = 0; i < text.size();) {
            int bytes = 1;
            const int codepoint = GetCodepointNext(text.c_str() + i, &bytes);
            i += std::max(1, bytes);
            const auto found = advances.find(codepoint);
            width += found != advances.end() ? found->second/base : .56f;
        }
        return width*px;
    }
    Measure Measurer() const { return [this](const std::string& text, float px) { return Width(text, px); }; }
private:
    static constexpr int base = 64;
    std::unordered_map<int, float> advances;
};

// ---- PDF -----------------------------------------------------------------

// Zlib (RFC 1950) stream around raylib's raw DEFLATE, as FlateDecode needs.
inline std::string ZlibCompress(const unsigned char* data, int size) {
    std::string out = "\x78\x01";
    int compressed = 0;
    unsigned char* raw = CompressData(data, size, &compressed);
    if (raw) { out.append(reinterpret_cast<const char*>(raw), compressed); MemFree(raw); }
    std::uint32_t a = 1, b = 0;
    for (int i = 0; i < size; ++i) { a = (a + data[i])%65521; b = (b + a)%65521; }
    const std::uint32_t adler = (b << 16) | a;
    for (int shift = 24; shift >= 0; shift -= 8) out += char((adler >> shift) & 0xFF);
    return out;
}

// Runs of one standard font: Helvetica (WinAnsi) or Symbol for Greek.
struct TextRun { bool symbol; std::string bytes; };
inline std::vector<TextRun> PdfRuns(const std::string& text) {
    std::vector<TextRun> runs;
    for (std::size_t i = 0; i < text.size();) {
        int length = 1;
        const int c = GetCodepointNext(text.c_str() + i, &length);
        i += std::max(1, length);
        char symbol = 0, latin = 0;
        switch (c) {
            case 0x03B8: symbol = 'q'; break;  // theta
            case 0x03C6: symbol = 'f'; break;  // phi
            case 0x03D5: symbol = 'j'; break;  // phi1
            case 0x03C1: symbol = 'r'; break;  // rho
            case 0x03C0: symbol = 'p'; break;  // pi
            case 0x2212: latin = '-'; break;
            case 0xB0: case 0xB7: case 0xD7: latin = char(c); break;
            default: latin = c >= 32 && c < 127 ? char(c) : '?';
        }
        const bool useSymbol = symbol != 0;
        if (runs.empty() || runs.back().symbol != useSymbol) runs.push_back({useSymbol, {}});
        runs.back().bytes += useSymbol ? symbol : latin;
    }
    return runs;
}
inline std::string PdfString(const std::string& bytes) {
    std::string out = "(";
    for (char c : bytes) { if (c == '(' || c == ')' || c == '\\') out += '\\'; out += c; }
    return out + ")";
}
inline std::string Num(double v) { char t[32]; std::snprintf(t, sizeof(t), "%.3f", v); return t; }
inline std::string Rgb(Color c) { return Num(c.r/255.0) + " " + Num(c.g/255.0) + " " + Num(c.b/255.0); }

// A one-page PDF: the scene image under vector strokes, fills and text.
// Coordinates are image pixels; the page is sized for dpi.
inline std::string BuildPdf(Vector2 canvas, float dpi, std::optional<Color> background, const Image& scene,
                            Vector2 sceneOffset, const Overlay& overlay, const TextMetrics& metrics) {
    const double scale = 72.0/dpi, width = canvas.x*scale, height = canvas.y*scale;
    // Opacity states, one per distinct alpha.
    std::map<unsigned char, int> alphas;
    auto state = [&](Color c) -> std::string {
        if (c.a == 255) return "";
        const auto [it, added] = alphas.try_emplace(c.a, int(alphas.size()) + 1);
        return "/A" + std::to_string(it->second) + " gs ";
    };
    std::ostringstream content;
    content << Num(scale) << " 0 0 " << Num(-scale) << " 0 " << Num(height) << " cm\n";
    if (background) content << Rgb(*background) << " rg 0 0 " << Num(canvas.x) << " " << Num(canvas.y) << " re f\n";
    if (scene.data) content << "q " << scene.width << " 0 0 " << -scene.height << " " << Num(sceneOffset.x) << " "
                            << Num(sceneOffset.y + scene.height) << " cm /Im1 Do Q\n";
    content << "1 J 1 j\n";
    for (const auto& f : overlay.fills)
        content << "q " << state(f.color) << Rgb(f.color) << " rg " << Num(f.box.x) << " " << Num(f.box.y) << " "
                << Num(f.box.width) << " " << Num(f.box.height) << " re f Q\n";
    for (const auto& s : overlay.strokes) {
        if (s.points.size() < 2) continue;
        content << "q " << state(s.color) << Rgb(s.color) << " RG " << Num(s.width) << " w ";
        if (s.dashed) content << "[" << Num(s.width*4 + 2) << " " << Num(s.width*3 + 2) << "] 0 d ";
        content << Num(s.points[0].x) << " " << Num(s.points[0].y) << " m";
        for (std::size_t i = 1; i < s.points.size(); ++i) content << " " << Num(s.points[i].x) << " " << Num(s.points[i].y) << " l";
        content << " S Q\n";
    }
    for (const auto& l : overlay.labels) {
        const float w = metrics.Width(l.plain, l.size);
        const float left = l.at.x - l.align.x*w, top = l.at.y - l.align.y*l.size;
        content << "q " << state(l.color) << Rgb(l.color) << " rg BT ";
        content << "1 0 0 -1 " << Num(left) << " " << Num(top + l.size*.78f) << " Tm ";
        for (const auto& run : PdfRuns(l.plain))
            content << (run.symbol ? "/F2 " : "/F1 ") << Num(l.size) << " Tf " << PdfString(run.bytes) << " Tj ";
        content << "ET Q\n";
    }
    const std::string text = content.str();
    const std::string stream = ZlibCompress(reinterpret_cast<const unsigned char*>(text.data()), int(text.size()));

    std::vector<std::string> objects;
    std::string resources = "<< /Font << /F1 5 0 R /F2 6 0 R >>";
    if (scene.data) resources += " /XObject << /Im1 7 0 R >>";
    if (!alphas.empty()) {
        resources += " /ExtGState <<";
        for (const auto& [alpha, id] : alphas)
            resources += " /A" + std::to_string(id) + " << /CA " + Num(alpha/255.0) + " /ca " + Num(alpha/255.0) + " >>";
        resources += " >>";
    }
    resources += " >>";
    objects.push_back("<< /Type /Catalog /Pages 2 0 R >>");
    objects.push_back("<< /Type /Pages /Kids [3 0 R] /Count 1 >>");
    objects.push_back("<< /Type /Page /Parent 2 0 R /MediaBox [0 0 " + Num(width) + " " + Num(height) + "] /Resources " +
                      resources + " /Contents 4 0 R >>");
    objects.push_back("<< /Length " + std::to_string(stream.size()) + " /Filter /FlateDecode >>\nstream\n" + stream + "\nendstream");
    objects.push_back("<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding >>");
    objects.push_back("<< /Type /Font /Subtype /Type1 /BaseFont /Symbol >>");
    if (scene.data) {
        // RGB samples, with an alpha mask only when something is see-through.
        const auto* pixels = static_cast<const unsigned char*>(scene.data);
        const std::size_t count = std::size_t(scene.width)*scene.height;
        std::string rgb(count*3, '\0'), alpha(count, '\0');
        bool translucent = false;
        for (std::size_t i = 0; i < count; ++i) {
            rgb[3*i] = char(pixels[4*i]); rgb[3*i + 1] = char(pixels[4*i + 1]); rgb[3*i + 2] = char(pixels[4*i + 2]);
            alpha[i] = char(pixels[4*i + 3]);
            translucent |= pixels[4*i + 3] != 255;
        }
        const std::string color = ZlibCompress(reinterpret_cast<const unsigned char*>(rgb.data()), int(rgb.size()));
        const std::string header = "<< /Type /XObject /Subtype /Image /Width " + std::to_string(scene.width) + " /Height " +
            std::to_string(scene.height) + " /BitsPerComponent 8 /Filter /FlateDecode /Length ";
        objects.push_back(header + std::to_string(color.size()) + " /ColorSpace /DeviceRGB" +
                          (translucent ? " /SMask 8 0 R" : "") + " >>\nstream\n" + color + "\nendstream");
        if (translucent) {
            const std::string mask = ZlibCompress(reinterpret_cast<const unsigned char*>(alpha.data()), int(alpha.size()));
            objects.push_back(header + std::to_string(mask.size()) + " /ColorSpace /DeviceGray >>\nstream\n" + mask + "\nendstream");
        }
    }
    std::string pdf = "%PDF-1.4\n%\xE2\xE3\xCF\xD3\n";
    std::vector<std::size_t> offsets;
    for (std::size_t i = 0; i < objects.size(); ++i) {
        offsets.push_back(pdf.size());
        pdf += std::to_string(i + 1) + " 0 obj\n" + objects[i] + "\nendobj\n";
    }
    const std::size_t xref = pdf.size();
    pdf += "xref\n0 " + std::to_string(objects.size() + 1) + "\n0000000000 65535 f \n";
    for (auto offset : offsets) { char line[32]; std::snprintf(line, sizeof(line), "%010zu 00000 n \n", offset); pdf += line; }
    pdf += "trailer\n<< /Size " + std::to_string(objects.size() + 1) + " /Root 1 0 R >>\nstartxref\n" + std::to_string(xref) + "\n%%EOF\n";
    return pdf;
}

// ---- LaTeX ---------------------------------------------------------------

struct LatexTools {
    std::map<std::string, std::string> engines;  // pdflatex, xelatex, lualatex -> path
    std::string dvi;                              // latex for DVI output (labels for PNG)
    std::string dvipng;
    bool Any() const { return !engines.empty(); }
    bool PngLabels() const { return !dvi.empty() && !dvipng.empty(); }

    // PATH plus the usual TeX locations: a viewer started from the Finder or
    // a desktop launcher often lacks the TeX directory on its PATH.
    static LatexTools Discover() {
        std::vector<std::filesystem::path> directories;
        if (const char* path = std::getenv("PATH")) {
            std::stringstream list(path);
            for (std::string entry; std::getline(list, entry, ':');) if (!entry.empty()) directories.push_back(entry);
        }
        for (const char* d : {"/Library/TeX/texbin", "/usr/texbin", "/opt/homebrew/bin", "/usr/local/bin", "/opt/local/bin", "/usr/bin"})
            directories.push_back(d);
        std::error_code ignored;
        for (const char* root : {"/usr/local/texlive", "/opt/texlive"}) {
            for (const auto& year : std::filesystem::directory_iterator(root, ignored)) {
                for (const auto& platform : std::filesystem::directory_iterator(year.path()/"bin", ignored))
                    directories.push_back(platform.path());
            }
        }
        auto find = [&](const std::string& name) -> std::string {
            for (const auto& directory : directories) {
                const auto candidate = directory/name;
                const auto permissions = std::filesystem::status(candidate, ignored).permissions();
                if (std::filesystem::is_regular_file(candidate, ignored) &&
                    (permissions & std::filesystem::perms::owner_exec) != std::filesystem::perms::none) return candidate.string();
            }
            return {};
        };
        LatexTools tools;
        for (const char* engine : {"pdflatex", "xelatex", "lualatex"})
            if (auto path = find(engine); !path.empty()) tools.engines[engine] = path;
        tools.dvi = find("latex");
        tools.dvipng = find("dvipng");
        return tools;
    }
};

inline std::string ShellQuote(const std::string& text) {
    std::string out = "'";
    for (char c : text) { if (c == '\'') out += "'\\''"; else out += c; }
    return out + "'";
}
inline std::string ReadText(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}
inline void WriteText(const std::filesystem::path& path, const std::string& text) {
    std::ofstream out(path, std::ios::binary);
    out.exceptions(std::ios::badbit | std::ios::failbit);
    out << text;
}
// The first LaTeX error and the lines after it, for the status line.
inline std::string LatexError(const std::filesystem::path& log) {
    const std::string text = ReadText(log);
    const auto at = text.find("\n! ");
    if (at == std::string::npos) return text.size() > 600 ? text.substr(text.size() - 600) : text;
    return text.substr(at + 1, 400);
}
inline std::filesystem::path TemporaryDirectory() {
    std::random_device random;
    char name[48];
    std::snprintf(name, sizeof(name), "rayfem-figure-%08x%08x", random(), random());
    auto path = std::filesystem::temp_directory_path()/name;
    std::filesystem::create_directories(path);
    return path;
}
inline int Run(const std::filesystem::path& directory, const std::string& command, const std::string& log) {
    return std::system(("cd " + ShellQuote(directory.string()) + " && " + command + " > " + ShellQuote(log) + " 2>&1 < /dev/null").c_str());
}

inline std::string TikzColor(Color c) {
    return "{rgb,255:red," + std::to_string(c.r) + ";green," + std::to_string(c.g) + ";blue," + std::to_string(c.b) + "}";
}
inline const char* TikzAnchor(Vector2 align) {
    static const char* names[3][3] = {{"north west", "north", "north east"}, {"west", "center", "east"}, {"south west", "south", "south east"}};
    auto index = [](float f) { return f < .25f ? 0 : f > .75f ? 2 : 1; };
    return names[index(align.y)][index(align.x)];
}
inline std::string LatexPreamble(const std::string& engine) {
    std::string preamble = "\\usepackage{amsmath}\n\\usepackage{lmodern}\n";
    if (engine == "pdflatex" || engine == "latex") preamble = "\\usepackage[T1]{fontenc}\n" + preamble;
    return preamble;
}

// A standalone TikZ picture: the scene image under vector overlay, labels
// typeset by LaTeX. One TikZ unit is one image pixel.
inline std::string BuildTikz(Vector2 canvas, float dpi, std::optional<Color> background, const std::string& sceneFile,
                             Vector2 sceneOffset, Vector2 sceneSize, const Overlay& overlay, const std::string& engine) {
    const double bp = 72.0/dpi;
    std::ostringstream tex;
    tex << "% Generated by RayFEM. One unit is one pixel of the " << int(canvas.x) << "x" << int(canvas.y) << " image at " << dpi << " dpi.\n";
    tex << "\\documentclass[tikz,border=0pt]{standalone}\n" << LatexPreamble(engine) << "\\begin{document}\n";
    tex << "\\begin{tikzpicture}[x=" << Num(bp) << "bp,y=-" << Num(bp) << "bp,line cap=round,line join=round]\n";
    tex << "\\useasboundingbox (0,0) rectangle (" << Num(canvas.x) << "," << Num(canvas.y) << ");\n";
    if (background) tex << "\\fill[fill=" << TikzColor(*background) << "] (0,0) rectangle (" << Num(canvas.x) << "," << Num(canvas.y) << ");\n";
    if (!sceneFile.empty())
        tex << "\\node[anchor=north west,inner sep=0pt] at (" << Num(sceneOffset.x) << "," << Num(sceneOffset.y) << ") {\\includegraphics[width="
            << Num(sceneSize.x*bp) << "bp,height=" << Num(sceneSize.y*bp) << "bp]{" << sceneFile << "}};\n";
    for (const auto& f : overlay.fills)
        tex << "\\fill[fill=" << TikzColor(f.color) << (f.color.a < 255 ? ",fill opacity=" + Num(f.color.a/255.0) : "") << "] ("
            << Num(f.box.x) << "," << Num(f.box.y) << ") rectangle (" << Num(f.box.x + f.box.width) << "," << Num(f.box.y + f.box.height) << ");\n";
    for (const auto& s : overlay.strokes) {
        if (s.points.size() < 2) continue;
        tex << "\\draw[draw=" << TikzColor(s.color) << ",line width=" << Num(s.width*bp) << "bp";
        if (s.color.a < 255) tex << ",draw opacity=" << Num(s.color.a/255.0);
        if (s.dashed) tex << ",dash pattern=on " << Num((s.width*4 + 2)*bp) << "bp off " << Num((s.width*3 + 2)*bp) << "bp";
        tex << "] (" << Num(s.points[0].x) << "," << Num(s.points[0].y) << ")";
        for (std::size_t i = 1; i < s.points.size(); ++i) tex << " -- (" << Num(s.points[i].x) << "," << Num(s.points[i].y) << ")";
        tex << ";\n";
    }
    for (const auto& l : overlay.labels) {
        const double size = l.size*bp;
        tex << "\\node[anchor=" << TikzAnchor(l.align) << ",inner sep=0pt,text=" << TikzColor(l.color)
            << (l.color.a < 255 ? ",text opacity=" + Num(l.color.a/255.0) : "") << ",font=\\fontsize{" << Num(size) << "}{"
            << Num(size*1.2) << "}\\selectfont] at (" << Num(l.at.x) << "," << Num(l.at.y) << ") {" << l.latex << "};\n";
    }
    tex << "\\end{tikzpicture}\n\\end{document}\n";
    return tex.str();
}

// Compile a TikZ document in directory; returns an error, or nothing.
inline std::optional<std::string> CompileLatex(const std::string& engine, const std::filesystem::path& directory,
                                               const std::string& name) {
    const int status = Run(directory, ShellQuote(engine) + " -interaction=nonstopmode -halt-on-error -no-shell-escape " +
                                          ShellQuote(name + ".tex"), "compile.log");
    if (status == 0 && std::filesystem::exists(directory/(name + ".pdf"))) return std::nullopt;
    const auto log = directory/(name + ".log");
    return "LaTeX failed: " + LatexError(std::filesystem::exists(log) ? log : directory/"compile.log");
}

// Typeset each label with LaTeX and rasterize it with dvipng at the figure's
// resolution, transparent around the ink. One image per label, in order.
inline std::vector<Image> RenderLatexLabels(const LatexTools& tools, const std::vector<Label>& labels, float dpi,
                                            const std::filesystem::path& directory, std::string& error) {
    std::vector<Image> images;
    if (labels.empty()) return images;
    const double bp = 72.0/dpi;
    std::ostringstream tex;
    tex << "\\documentclass{article}\n" << LatexPreamble("latex") << "\\usepackage{xcolor}\n\\pagestyle{empty}\n\\begin{document}\n";
    for (const auto& l : labels) {
        const double size = l.size*bp;
        tex << "\\noindent\\fontsize{" << Num(size) << "}{" << Num(size*1.2) << "}\\selectfont\\color[RGB]{" << int(l.color.r) << ","
            << int(l.color.g) << "," << int(l.color.b) << "}" << (l.latex.empty() ? "\\mbox{}" : l.latex) << "\n\\newpage\n";
    }
    tex << "\\end{document}\n";
    WriteText(directory/"labels.tex", tex.str());
    if (Run(directory, ShellQuote(tools.dvi) + " -interaction=nonstopmode -halt-on-error -no-shell-escape labels.tex", "labels-latex.log") != 0) {
        error = "LaTeX failed on a label: " + LatexError(directory/"labels.log");
        return images;
    }
    char resolution[32];
    std::snprintf(resolution, sizeof(resolution), "%.2f", dpi);
    if (Run(directory, ShellQuote(tools.dvipng) + " -q -T tight -D " + resolution + " -bg Transparent -o label%d.png labels.dvi",
            "dvipng.log") != 0) {
        error = "dvipng failed: " + ReadText(directory/"dvipng.log");
        return images;
    }
    for (std::size_t i = 0; i < labels.size(); ++i) {
        Image image = LoadImage((directory/("label" + std::to_string(i + 1) + ".png")).string().c_str());
        if (image.data) ImageFormat(&image, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);
        images.push_back(image);
    }
    return images;
}

// Draw label images at their anchors over an image (alpha blended).
inline void CompositeLabels(Image& canvas, const std::vector<Label>& labels, const std::vector<Image>& images) {
    for (std::size_t i = 0; i < labels.size() && i < images.size(); ++i) {
        const Image& label = images[i];
        if (!label.data) continue;
        const float x = labels[i].at.x - labels[i].align.x*label.width, y = labels[i].at.y - labels[i].align.y*label.height;
        ImageDraw(&canvas, label, {0, 0, float(label.width), float(label.height)},
                  {std::round(x), std::round(y), float(label.width), float(label.height)}, WHITE);
    }
}

} // namespace RayFEM::UI::Figure
