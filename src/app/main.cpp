#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "CLI/CLI.hpp"
#include "mfem.hpp"
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include "imgui.h"
#include "rlImGui.h"
#include "czmq.h"
#include "libRayfem/network/server.hpp"
#include "libRayfem/mesh/converter.hpp"
#include "libRayfem/mesh/clipping.hpp"
#include "libRayfem/mesh/field_range.hpp"
#include "libRayfem/io/serif.hpp"
#include "file_dialog.hpp"
#include "theme.hpp"
#include "profile_panel.hpp"
#include "camera_controller.hpp"
#include "orientation_cube.hpp"
#include "reference_plane.hpp"
#include "rotation_icons.hpp"
#include "tracking_panel.hpp"
#include "surface_trace_panel.hpp"
#include "flow_panel.hpp"
#include "edge_renderer.hpp"
#include "element_picker.hpp"
#include "element_panel.hpp"
#include "mouse_modifiers.hpp"

struct Options {
    std::string listen = "127.0.0.1";
    int port = 5555;
    int controlPort = 0;
    int fps = 60;
    int width = 1440;
    int height = 900;
    int history = 120;
    std::string archive;
    bool light = false;
    float explosion = 0;
};

namespace {
constexpr const char* viewNames[] = {"Value", "Magnitude", "Component", "Divergence", "Curl magnitude", "Curl component"};
constexpr const char* sliceFragment = R"(#version 330
in vec3 fragPosition;
in vec3 fragNormal;
in vec2 fragTexCoord;
in vec4 fragColor;
uniform vec4 colDiffuse;
uniform vec3 planeNormal;
uniform vec3 planePoint;
uniform int clipEnabled;
uniform float overlayDepthBias;
out vec4 finalColor;
void main() {
    if (clipEnabled != 0 && dot(fragPosition-planePoint,planeNormal)>0.0) discard;
    float light=0.45+0.55*abs(dot(normalize(fragNormal),normalize(vec3(0.4,0.6,0.7))));
    finalColor=vec4(fragColor.rgb*colDiffuse.rgb*light,fragColor.a*colDiffuse.a);
    // Slope-aware polygon offset keeps line rasterization above the adjacent
    // triangles without revealing hidden edges on the back of the mesh.
    float slope=max(abs(dFdx(gl_FragCoord.z)),abs(dFdy(gl_FragCoord.z)));
    gl_FragDepth=min(1.0,gl_FragCoord.z+overlayDepthBias*(slope+0.000001));
})";
constexpr const char* sliceVertex = R"(#version 330
in vec3 vertexPosition;
in vec3 vertexNormal;
in vec2 vertexTexCoord;
in vec4 vertexColor;
uniform mat4 mvp;
out vec3 fragPosition;
out vec3 fragNormal;
out vec2 fragTexCoord;
out vec4 fragColor;
void main() {
    fragPosition=vertexPosition;
    fragNormal=vertexNormal;
    fragTexCoord=vertexTexCoord;
    fragColor=vertexColor;
    gl_Position=mvp*vec4(vertexPosition,1.0);
})";

struct ViewCache {
    std::shared_ptr<RayFEM::RenderFrame> frame;
    std::string field;
    std::string clipping;
    int view = 0, component = 0, refinement = 3;
    float displacement = 1;
    Model model{};
    Model edgeModel{};
    Model selectionModel{};
    RayFEM::UI::ElementPicker picker;
    std::uint64_t selectionRevision=UINT64_MAX;
    std::size_t selectionVertices=0;
    std::string appearance;
    std::vector<double> values;
    std::vector<double> rangeDistribution;
    BoundingBox bounds{};
    double min = 0, max = 0, build_ms = 0;
    size_t vertices = 0;
    size_t rangeSamples = 0;
    bool rangeBudgetLimited = false;
    bool hasFieldRange = false;
    RayFEM::ColorScaleMapping colorMapping{};
    double volumeMin=0,volumeMax=0;
    std::vector<RayFEM::DofPoint> dofs;
    std::vector<RayFEM::LineSegment> boundaries;
    std::vector<bool> retainedElements;
    bool boundaries_ready = false;
    bool edges_uploaded = false;
    bool dofs_ready = false;
};

struct RangeCache {
    std::weak_ptr<RayFEM::RenderFrame> frame;
    std::string field;
    RayFEM::FieldView view;
    int component;
    std::vector<int> excluded;
    std::vector<int> excludedElements;
    RayFEM::FieldRange range;
};
}

class VisualizerApp {
public:
    explicit VisualizerApp(const Options& options) : opts(options), server(options.listen, options.port,
            options.controlPort ? options.controlPort : options.port + 1) {
        const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
        while(!server.ready() && !server.getErrorCount() && std::chrono::steady_clock::now()<deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        if(!server.ready()) throw std::runtime_error(server.getLastError().empty()?"Timed out starting the stream listener":server.getLastError());
        SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
        InitWindow(opts.width, opts.height, "RayFEM | fields in motion");
        if(!IsWindowReady()) throw std::runtime_error("Could not initialize the RayFEM window");
        RayFEM::UI::InitMouseModifiers();
        SetWindowMinSize(960, 640);
        SetTargetFPS(opts.fps);
        elementExplosion=opts.explosion;
        rlImGuiSetup(true);
        ImGui::GetIO().IniFilename = nullptr;
        ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
        ImFontConfig font;
        font.SizePixels=16;
        ImFont* uiFont=nullptr;
        // Use an installed system face where available, with an embedded
        // fallback so no font download or platform dependency is required.
        for(const char* path:{"/System/Library/Fonts/Supplemental/Arial.ttf",
                              "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                              "C:/Windows/Fonts/segoeui.ttf"}) {
            if(std::filesystem::is_regular_file(path)) {
                uiFont=ImGui::GetIO().Fonts->AddFontFromFileTTF(path,16,&font);
                if(uiFont) break;
            }
        }
        ImGui::GetIO().FontDefault=uiFont?uiFont:ImGui::GetIO().Fonts->AddFontDefault(&font);
        RayFEM::UI::ApplyLayout();
        RayFEM::UI::ApplyTheme(opts.light ? RayFEM::UI::ThemeMode::Light : RayFEM::UI::ThemeMode::Dark);
        rotationIcons=RayFEM::UI::LoadRotationIcons();
        shader = LoadShaderFromMemory(sliceVertex, sliceFragment);
        edgeRenderer.Initialize();
        streamlineRenderer.Initialize();
        normalLoc = GetShaderLocation(shader, "planeNormal");
        pointLoc = GetShaderLocation(shader, "planePoint");
        enabledLoc = GetShaderLocation(shader, "clipEnabled");
        depthBiasLoc = GetShaderLocation(shader, "overlayDepthBias");
        const auto home=RayFEM::ReferencePlaneHomeBasis(referenceRotation);
        camera = {Vector3Scale(home.back,12), {0, 0, 0}, home.up, 40, CAMERA_PERSPECTIVE};
        if (!opts.archive.empty()) LoadArchive(opts.archive);
        server.markViewerReady();
    }

    ~VisualizerApp() {
        server.markViewerReady(false);
        ClearCache();
        flow.ClearGraphics();
        streamlineRenderer.Shutdown();
        edgeRenderer.Shutdown();
        ranges.clear();
        if (target.id) UnloadRenderTexture(target);
        UnloadShader(shader);
        RayFEM::UI::UnloadRotationIcons(rotationIcons);
        rlImGuiShutdown();
        RayFEM::UI::ShutdownMouseModifiers();
        CloseWindow();
    }

    void Run() {
        while (!WindowShouldClose() && !zsys_interrupted && !server.shutdownRequested()) {
            Ingest();
            HandleDroppedFiles();
            BeginDrawing();
            ClearBackground(RayFEM::UI::Palette().canvas_background);
            rlImGuiBegin();
            DrawUI();
            rlImGuiEnd();
            EndDrawing();
        }
    }

private:
    std::shared_ptr<RayFEM::RenderFrame> Current() const {
        return selected >= 0 && selected < static_cast<int>(frames.size()) ? frames[selected] : nullptr;
    }

    mfem::GridFunction* Field(const std::shared_ptr<RayFEM::RenderFrame>& frame) {
        if (!frame || frame->fields.empty()) return nullptr;
        auto it = std::find_if(frame->fields.begin(), frame->fields.end(), [&](const auto& f) { return f.name == fieldName; });
        if (it == frame->fields.end()) {
            it = frame->fields.begin();
            fieldName = it->name;
            view = component = 0;
            automaticColorRange = true;
        }
        return it->gf.get();
    }

    void LoadArchive(const std::filesystem::path& path) {
        auto loaded = RayFEM::LoadSerifFile(path);
        archiveError = loaded.error;
        archiveReportWarnings = std::move(loaded.warnings);
        if (!loaded) {
            archiveReport = true;
            std::cerr << "RayFEM: " << archiveError << '\n';
            return;
        }
        // Commit only after the complete import succeeds. A failed open keeps
        // the user's current mesh, fields, and analysis available.
        ClearCache();
        ranges.clear();
        automaticColorRange=true;
        profiles = ProfilePanel{};
        tracking = TrackingPanel{};
        surfaceTrace = SurfaceTracePanel{};
        flow = FlowPanel{};
        elements = ElementPanel{};
        frames.assign(loaded.frames.begin(), loaded.frames.end());
        archivePath = path.string();
        archiveWarnings = archiveReportWarnings;
        selected = 0;
        follow = playing = false;
        view = component = coloring = 0;
        fieldName.clear();
        error.clear();
        failedKey.clear();
        excludedMaterials.clear();
        cameraAdjusted = clip = planeGuide = pickingDof = false;
        cameraTransition.active=false;
        cameraDrag={};cubePointer={};
        fittedMesh.reset();
        showSurface = deform = true;
        showDofs = false;
        displacementScale = 1;
        Field(Current());
        archiveReport = !archiveWarnings.empty();
        std::cout << "Opened " << path << ": " << frames.size() << " mesh view(s)\n";
    }

    void HandleDroppedFiles() {
        if (!IsFileDropped()) return;
        const auto files = LoadDroppedFiles();
        // One archive can already hold several mesh/field views. Open one
        // archive atomically so accidental multi-selection cannot overwrite it.
        if (files.count == 1) LoadArchive(files.paths[0]);
        else {
            archiveError = "Drop one .serif archive at a time.";
            archiveReportWarnings.clear();
            archiveReport = true;
        }
        UnloadDroppedFiles(files);
    }

    void DrawArchiveDialogs() {
        if (auto path = fileDialog.Draw(RayFEM::SerifLoadingAvailable())) LoadArchive(*path);
        if (archiveReport) {
            ImGui::OpenPopup("Archive details");
            archiveReport = false;
        }
        ImGui::SetNextWindowSize({620, 430}, ImGuiCond_Appearing);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding,RayFEM::UI::OverlayRounding);
        const bool reportVisible=ImGui::BeginPopupModal("Archive details", nullptr, ImGuiWindowFlags_NoSavedSettings);
        ImGui::PopStyleVar();
        if (reportVisible) {
            ImGui::BeginChild("Import messages", {0,-45});
            if (!archiveError.empty())
                ImGui::TextWrapped("%s", archiveError.c_str());
            else {
                ImGui::TextUnformatted("Archive opened");
                ImGui::TextDisabled("%s", std::filesystem::path(archivePath).filename().string().c_str());
            }
            for (const auto& warning : archiveReportWarnings) {
                ImGui::Spacing();
                ImGui::TextWrapped("%s", warning.c_str());
            }
            ImGui::EndChild();
            ImGui::Spacing();
            if (ImGui::Button("Close", {90, 0})) ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }
    }

    void Ingest() {
        const auto previous = Current();
        for (auto& frame : server.fetchFrames()) {
            tracking.Observe(frame);
            frames.push_back(std::move(frame));
        }
        while (frames.size() > static_cast<size_t>(opts.history) || (frames.size()>1 && HistoryBytes()>256ULL*1024*1024)) {
            auto oldest = frames.front();
            for (auto it=cache.begin(); it!=cache.end();) {
                if (it->frame == oldest) { ReleaseView(*it); it=cache.erase(it); }
                else ++it;
            }
            frames.pop_front();
            --selected;
        }
        if (follow && !frames.empty()) selected = static_cast<int>(frames.size()) - 1;
        else if (!frames.empty()) selected = std::clamp(selected, 0, static_cast<int>(frames.size()) - 1);
        if (playing && !frames.empty() && GetTime() - lastPlay >= 1.0 / playbackFps) {
            selected = (selected + 1) % frames.size();
            lastPlay = GetTime();
        }
        if (Current() != previous) error.clear();
    }

    size_t HistoryBytes() const {
        return RayFEM::EstimateHistoryBytes(frames);
    }

    RayFEM::RenderOptions RenderSettings(const std::shared_ptr<RayFEM::RenderFrame>& frame) const {
        RayFEM::RenderOptions o;
        o.refinement = automaticDetail && frame && frame->mesh ?
            RayFEM::RecommendedSurfaceRefinement(*frame->mesh,
                deform ? frame->displacement.get() : nullptr) : refinement;
        o.view = static_cast<RayFEM::FieldView>(view);
        o.component = component;
        o.coloring=static_cast<RayFEM::SurfaceColoring>(coloring);
        o.color_map=static_cast<RayFEM::ColorMap>(colorMap);
        o.reverse_colors=reverseColors;
        o.color_scale=colorScale;
        o.excluded_materials=excludedMaterials;
        o.excluded_elements=elements.Hidden();
        if (frame && frame->mesh) {
            for (const auto& field : frame->fields) if (field.name == fieldName) {
                RayFEM::ExcludeUnsupportedMaterials(field, *frame->mesh, o.excluded_materials);
                break;
            }
        }
        o.displacement = deform && frame ? frame->displacement.get() : nullptr;
        o.displacement_scale = displacementScale;
        o.element_explosion = elementExplosion;
        return o;
    }

    static void ReleaseView(ViewCache& entry) {
        UnloadModel(entry.model);
        UnloadModel(entry.edgeModel);
        UnloadModel(entry.selectionModel);
    }

    void ClearCache() {
        for (auto& entry : cache) ReleaseView(entry);
        cache.clear();
    }

    void TrimViewCache() {
        auto bytes=[](const ViewCache& entry) {
            return entry.vertices*72+entry.boundaries.size()*464+entry.picker.StorageBytes()+entry.selectionVertices*64+
                entry.dofs.size()*sizeof(RayFEM::DofPoint);
        };
        std::size_t total=0;
        for(const auto& entry:cache)total+=bytes(entry);
        // ActiveView always makes the displayed entry the newest. Preserve it
        // even when one detailed view alone exceeds the retention budget.
        while(cache.size()>6 || (cache.size()>1 && total>128ULL*1024*1024)) {
            total-=bytes(cache.front());ReleaseView(cache.front());cache.pop_front();
        }
    }

    std::string AppearanceKey() const {
        std::ostringstream key;
        key<<colorMap<<','<<reverseColors<<','<<automaticColorRange<<','
           <<static_cast<int>(colorScale.scale)<<','<<std::setprecision(17)
           <<colorScale.log_threshold<<','<<colorScale.zscale_contrast<<','
           <<colorMinimum<<','<<colorMaximum;
        return key.str();
    }

    void RefreshAppearance(ViewCache& entry,const std::string& key) {
        if(entry.appearance==key)return;
        if(entry.hasFieldRange) {
            const auto mapping=automaticColorRange ?
                RayFEM::ResolveColorScale(entry.volumeMin,entry.volumeMax,colorScale,entry.rangeDistribution) :
                RayFEM::ResolveColorScale(colorMinimum,colorMaximum,colorScale);
            if(entry.model.meshCount) {
                auto& mesh=entry.model.meshes[0];
                for(std::size_t i=0;i<entry.values.size();++i) {
                    const auto color=RayFEM::FieldColor(entry.values[i],mapping,
                        static_cast<RayFEM::ColorMap>(colorMap),reverseColors);
                    mesh.colors[4*i]=color.r;mesh.colors[4*i+1]=color.g;
                    mesh.colors[4*i+2]=color.b;mesh.colors[4*i+3]=color.a;
                }
                UpdateMeshBuffer(mesh,3,mesh.colors,static_cast<int>(entry.values.size()*4),0);
            }
            entry.colorMapping=mapping;entry.min=mapping.minimum;entry.max=mapping.maximum;
        }
        entry.appearance=key;
    }

    RayFEM::FieldRange VolumeRange(const std::shared_ptr<RayFEM::RenderFrame>& frame,
                                  mfem::GridFunction& field) {
        const auto options = RenderSettings(frame);
        std::erase_if(ranges, [](const auto& item) { return item.frame.expired(); });
        auto found = std::find_if(ranges.begin(), ranges.end(), [&](const auto& item) {
            return item.frame.lock()==frame && item.field==fieldName &&
                item.view==options.view && item.component==options.component &&
                item.excluded==options.excluded_materials && item.excludedElements==options.excluded_elements;
        });
        if (found!=ranges.end()) {
            std::rotate(found,found+1,ranges.end());
            return ranges.back().range;
        }
        RayFEM::FieldRangeOptions sampling;
        sampling.distribution_samples=1000;
        auto range=RayFEM::EvaluateFieldRange(*frame->mesh,field,options,sampling);
        ranges.push_back({frame,fieldName,options.view,options.component,options.excluded_materials,options.excluded_elements,range});
        if(ranges.size()>64) ranges.pop_front();
        return range;
    }

    ViewCache* ActiveView() {
        auto frame = Current();
        elements.Context(frame);
        if (!frame) return nullptr;
        auto* gf = Field(frame);
        const int surfaceRefinement = RenderSettings(frame).refinement;
        float amount = deform && frame->displacement ? displacementScale : 0;
        std::ostringstream clipDescription;
        clipDescription.precision(9);
        clipDescription<<"color="<<coloring<<";explode="<<elementExplosion<<";exclude=";
        for(int id:excludedMaterials) clipDescription<<id<<',';
        clipDescription<<";hidden=";
        for(int id:elements.Hidden()) clipDescription<<id<<',';
        clipDescription<<';';
        if (clip) clipDescription<<clipMode<<','<<cutAlgorithm<<','<<slicePoint.x<<','<<slicePoint.y<<','<<slicePoint.z<<','<<sliceNormal.x<<','<<sliceNormal.y<<','<<sliceNormal.z;
        const std::string clipKey=clipDescription.str();
        const std::string appearanceKey=AppearanceKey();
        auto it = std::find_if(cache.begin(), cache.end(), [&](const auto& e) {
            return e.frame==frame && e.field==fieldName && e.view==view && e.component==component && e.refinement==surfaceRefinement && e.displacement==amount && e.clipping==clipKey;
        });
        if (it != cache.end()) {
            std::rotate(it, it+1, cache.end());
            try {RefreshAppearance(cache.back(),appearanceKey);}
            catch(const std::exception& e) {error=e.what();return &cache.back();}
            if (!failedKey.empty()) {error.clear();failedKey.clear();}
            return &cache.back();
        }
        // Invalid field operations are shown once instead of retried every screen refresh.
        std::string key = std::to_string(reinterpret_cast<uintptr_t>(frame.get()))+fieldName+":"+std::to_string(view)+":"+std::to_string(component)+":"+std::to_string(surfaceRefinement)+":"+std::to_string(amount)+":"+clipKey+appearanceKey;
        if (failedKey==key) return nullptr;
        try {
            auto start=std::chrono::steady_clock::now();
            ViewCache entry;
            RayFEM::SurfaceData data;
            if (clip) {
                auto cut=RayFEM::BuildClippedSurface(*frame->mesh,gf,RenderSettings(frame),{slicePoint,sliceNormal},static_cast<RayFEM::ClipMode>(clipMode),static_cast<RayFEM::CutAlgorithm>(cutAlgorithm));
                data=std::move(cut.surface);entry.boundaries=std::move(cut.element_boundaries);entry.boundaries_ready=true;
                entry.retainedElements=std::move(cut.retained_elements);
            } else data=RayFEM::BuildSurface(*frame->mesh,gf,RenderSettings(frame));
            // Keep the legend and its scale even when clipping hides the
            // entire field. An explicitly empty material selection has none.
            const auto rangeOptions=RenderSettings(frame);
            bool hasIncludedElement=false;
            if(coloring==0 && gf) {
                const RayFEM::ElementSelection included(*frame->mesh,rangeOptions);
                for(int e=0;e<frame->mesh->GetNE();++e)
                    if(included.Includes(e)) {hasIncludedElement=true;break;}
            }
            if(hasIncludedElement) {
                const auto range=VolumeRange(frame,*gf);
                entry.colorMapping=automaticColorRange ?
                    RayFEM::ResolveColorScale(range.min_value,range.max_value,colorScale,range.samples) :
                    RayFEM::ResolveColorScale(colorMinimum,colorMaximum,colorScale);
                RayFEM::ApplyFieldColorRange(data,entry.colorMapping,static_cast<RayFEM::ColorMap>(colorMap),reverseColors);
                entry.volumeMin=range.min_value;entry.volumeMax=range.max_value;
                entry.rangeSamples=range.sample_count;
                entry.rangeBudgetLimited=range.budget_limited;
                entry.rangeDistribution=range.samples;
                entry.hasFieldRange=true;
            }
            entry.frame=frame; entry.field=fieldName; entry.view=view; entry.component=component;
            entry.clipping=clipKey;
            entry.refinement=surfaceRefinement; entry.displacement=amount;
            entry.bounds=data.bounds; entry.min=data.min_value; entry.max=data.max_value;
            entry.vertices=data.positions.size()/3;
            if(!data.positions.empty()) {
                entry.model=RayFEM::UploadSurface(data);
                entry.picker.Build({entry.model.meshes[0].vertices,data.positions.size()},data.element_ids);
            }
            if(entry.hasFieldRange)entry.values=std::move(data.values);
            entry.appearance=appearanceKey;
            for (int i=0;i<entry.model.materialCount;++i) entry.model.materials[i].shader=shader;
            entry.build_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
            cache.push_back(std::move(entry));
            TrimViewCache();
            if (cache.back().vertices && fittedMesh.lock() != frame->mesh) {
                if (!cameraAdjusted) FitCamera(cache.back().bounds,!clip);
                fittedMesh=frame->mesh;
            }
            error.clear(); failedKey.clear();
            return &cache.back();
        } catch (const std::exception& e) { error=e.what(); failedKey=key; return nullptr; }
    }

    void FitCamera(BoundingBox box,bool initializePlane=false) {
        cameraTransition.active=false;
        cameraDrag={};
        camera.target=Vector3Scale(Vector3Add(box.min,box.max),0.5f);
        radius=std::max(0.05f,Vector3Length(Vector3Subtract(box.max,box.min))*0.5f);
        const auto basis=RayFEM::ReferencePlaneHomeBasis(referenceRotation);
        camera.position=Vector3Add(camera.target,Vector3Scale(basis.back,radius*3.4f));
        camera.up=basis.up;
        if (initializePlane) slicePoint=camera.target;
    }

    void HomeCamera() {
        cameraDrag={};
        if(RayFEM::BeginCameraHomeTransition(cameraTransition,camera,referenceRotation,GetTime()))
            cameraAdjusted=true;
    }

    void RefreshSelection(ViewCache& entry) {
        if(entry.selectionRevision==elements.Revision()) return;
        UnloadModel(entry.selectionModel);entry.selectionModel={};entry.selectionVertices=0;
        entry.selectionRevision=elements.Revision();
        if(!entry.model.meshCount || elements.Selected().empty()) return;
        RayFEM::SurfaceData overlay;
        const auto& source=entry.model.meshes[0];
        std::size_t triangles=0;
        for(int id:elements.Selected())triangles+=entry.picker.TrianglesForElement(id).size();
        overlay.positions.reserve(triangles*9);overlay.normals.reserve(triangles*9);
        overlay.colors.reserve(triangles*12);
        for(int id:elements.Selected()) for(auto triangle:entry.picker.TrianglesForElement(id)) {
            const auto offset=std::size_t(triangle)*9;
            overlay.positions.insert(overlay.positions.end(),source.vertices+offset,source.vertices+offset+9);
            overlay.normals.insert(overlay.normals.end(),source.normals+offset,source.normals+offset+9);
            for(int vertex=0;vertex<3;++vertex)
                overlay.colors.insert(overlay.colors.end(),{255,189,60,150});
        }
        entry.selectionModel=RayFEM::UploadSurface(overlay);
        entry.selectionVertices=overlay.positions.size()/3;
        for(int i=0;i<entry.selectionModel.materialCount;++i)entry.selectionModel.materials[i].shader=shader;
        TrimViewCache();
    }

    static void Tip(const char* text) {
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) ImGui::SetTooltip("%s",text);
    }

    void DrawUI() {
        ImGui::SetNextWindowPos({0,0});
        ImGui::SetNextWindowSize({static_cast<float>(GetScreenWidth()),static_cast<float>(GetScreenHeight())});
        ImGui::Begin("RayFEM",nullptr,ImGuiWindowFlags_NoDecoration|ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoSavedSettings);
        const bool light = RayFEM::UI::CurrentTheme() == RayFEM::UI::ThemeMode::Light;
        const char* themeLabel=light?"Dark mode":"Light mode";
        const char* statusLabel=server.ready()?"Stream ready":"Connecting";
        const auto& style=ImGui::GetStyle();
        auto buttonWidth=[&](const char* label) {return ImGui::CalcTextSize(label).x+style.FramePadding.x*2;};
        const float actionsWidth=buttonWidth("Open")+buttonWidth(themeLabel)+buttonWidth("Info")+
            ImGui::CalcTextSize(statusLabel).x+3*style.ItemSpacing.x;
        const float actionsX=ImGui::GetCursorPosX()+ImGui::GetContentRegionAvail().x-actionsWidth;
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(RayFEM::UI::ThemeText(RayFEM::UI::Palette().accent),"RayFEM");
        ImGui::SameLine(112);
        std::string title = Current() ? fieldName : "Workspace";
        const float titleWidth = std::max(20.0f, actionsX-112-style.ItemSpacing.x-ImGui::CalcTextSize("/  ").x);
        const bool shorten = ImGui::CalcTextSize(title.c_str()).x > titleWidth;
        if (shorten) {
            while (!title.empty() && ImGui::CalcTextSize((title+"...").c_str()).x > titleWidth) {
                size_t end = title.size()-1;
                while (end && (static_cast<unsigned char>(title[end]) & 0xc0) == 0x80) --end;
                title.resize(end);
            }
            title += "...";
        }
        ImGui::TextDisabled("/  %s",title.c_str());
        if (shorten) Tip(fieldName.c_str());
        ImGui::SameLine(actionsX);
        if (ImGui::Button("Open")) fileDialog.Open();
        Tip("Open a .serif archive (Ctrl/Cmd+O). Files can also be dropped into the window.");
        ImGui::SameLine();
        if (ImGui::Button(themeLabel))
            RayFEM::UI::ApplyTheme(light ? RayFEM::UI::ThemeMode::Dark : RayFEM::UI::ThemeMode::Light);
        Tip("Switch the workspace, plots, and mesh overlays between light and dark.");
        ImGui::SameLine();
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(RayFEM::UI::ThemeText(server.ready()?RayFEM::UI::Palette().success:RayFEM::UI::Palette().warning),"%s",statusLabel);
        ImGui::SameLine();
        if (ImGui::Button("Info")) ImGui::OpenPopup("Session info");
        if (ImGui::BeginPopup("Session info")) {
            ImGui::Text("%s:%d",opts.listen.c_str(),opts.port);
            ImGui::Text("%zu received / %zu queued",server.getReceivedFrameCount(),server.getPendingFrameCount());
            ImGui::Text("%zu dropped / %zu errors",server.getDroppedFrameCount(),server.getErrorCount());
            ImGui::Text("%zu retained frames / ~%.1f MiB",frames.size(),HistoryBytes()/1048576.0);
            ImGui::Text("%d fps",GetFPS());
            if (!cache.empty()) ImGui::Text("%zu triangles / %.1f ms build",cache.back().vertices/3,cache.back().build_ms);
            if (!server.getLastError().empty()) ImGui::TextWrapped("%s",server.getLastError().c_str());
            ImGui::Text("SERiF files: %s", RayFEM::SerifLoadingAvailable()?"available":"not enabled in this build");
            if (!archivePath.empty()) {
                ImGui::Separator();
                ImGui::TextWrapped("%s", archivePath.c_str());
                for (const auto& warning : archiveWarnings) ImGui::TextWrapped("%s", warning.c_str());
            }
            ImGui::Separator();
            ImGui::TextUnformatted("Click to select  /  Shift-click to add or remove\nDrag to orbit  /  Middle-drag to pan\nScroll to zoom  /  R to fit\nClick the orientation cube to align\nCube arrows: 5° / double arrows: 45°\nArrow keys to step frames");
            ImGui::EndPopup();
        }
        if ((ImGui::GetIO().KeyCtrl || ImGui::GetIO().KeySuper) && ImGui::IsKeyPressed(ImGuiKey_O))
            fileDialog.Open();
        DrawArchiveDialogs();
        ImGui::Spacing();
        auto frame=Current();auto* gf=Field(frame);
        elements.Context(frame);
        profiles.UpdateContext(frame,gf,fieldName,RenderSettings(frame));
        surfaceTrace.UpdateContext(frame,gf,fieldName,RenderSettings(frame));
        flow.UpdateContext(frame,gf,fieldName,RenderSettings(frame));
        tracking.SetExcludedMaterials(excludedMaterials,frames);
        tracking.SetExcludedElements(elements.Hidden(),frame,frames);
        tracking.ProcessPending();
        ImGui::BeginChild("Inspector",{300,0},ImGuiChildFlags_Borders);
        ImGui::PushItemWidth(164);
        if (ImGui::BeginTabBar("Inspector tabs")) {
            if (ImGui::BeginTabItem("View",nullptr,inspectorTab==0?ImGuiTabItemFlags_SetSelected:0)) {
                if(inspectorTab==0) inspectorTab=-1;
                DrawControls();ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Analyze",nullptr,inspectorTab==1?ImGuiTabItemFlags_SetSelected:0)) {
                if(inspectorTab==1) inspectorTab=-1;
                profiles.Controls(frame,gf,fieldName,RenderSettings(frame));
                ImGui::Spacing();
                surfaceTrace.Controls(frame,gf,fieldName,RenderSettings(frame));
                ImGui::Spacing();
                tracking.Controls(frame,fieldName,RenderSettings(frame),frames);
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Elements",nullptr,inspectorTab==2?ImGuiTabItemFlags_SetSelected:0)) {
                if(inspectorTab==2) inspectorTab=-1;
                elements.Controls(frame,gf,fieldName,RenderSettings(frame));
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
        if(tracking.TakePlotRequest()) {plotTab=1;showPlots=true;}
        ImGui::PopItemWidth();
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginChild("Workspace",{0,0},ImGuiChildFlags_AlwaysUseWindowPadding);
        auto nextAction = [](const char* label, bool checkbox = false) {
            const auto& style = ImGui::GetStyle();
            const float width = ImGui::CalcTextSize(label).x + style.FramePadding.x*2 +
                                (checkbox ? ImGui::GetFrameHeight() : 0);
            RayFEM::UI::NextInlineItem(width);
        };
        if(ImGui::Button("Fit")) {cameraAdjusted=false;if(auto* active=ActiveView();active && active->vertices) FitCamera(active->bounds);}
        Tip("Fit mesh in view (R)");
        nextAction(pickingDof?"Cancel pick":"Pick DOF");
        if(ImGui::Button(pickingDof?"Cancel pick":"Pick DOF")) {pickingDof=!pickingDof;if(pickingDof) showDofs=true;}
        Tip("Pick a degree of freedom in the mesh to track its coefficient over time.");
        nextAction("1D profile");
        if(ImGui::Button("1D profile")) {profiles.OpenRay();surfaceTrace.SetEnabled(false);inspectorTab=1;plotTab=0;showPlots=true;}
        nextAction("2D profile");
        if(ImGui::Button("2D profile")) {profiles.OpenPlane();surfaceTrace.SetEnabled(false);inspectorTab=1;plotTab=0;showPlots=true;}
        nextAction("Surface trace");
        if(ImGui::Button("Surface trace")) {surfaceTrace.Open();profiles.SetMode(0);inspectorTab=1;plotTab=2;showPlots=true;}
        nextAction("Tracks");
        if(ImGui::Button("Tracks")) {inspectorTab=1;plotTab=1;showPlots=true;}
        nextAction("Flow");
        if(ImGui::Button("Flow")) {flow.Open();inspectorTab=0;}
        nextAction("Plots", true);ImGui::Checkbox("Plots",&showPlots);
        const float available=ImGui::GetContentRegionAvail().y;
        const bool plotVisible=showPlots && (profiles.Enabled() || surfaceTrace.Enabled() || tracking.HasTracks() || plotTab==1);
        const float plotHeight=plotVisible?std::clamp(available*0.40f,200.0f,320.0f):0;
        const float timelineHeight=TimelineHeight(ImGui::GetContentRegionAvail().x);
        DrawViewport({ImGui::GetContentRegionAvail().x,std::max(120.0f,available-timelineHeight-(plotVisible?plotHeight+style.ItemSpacing.y:0))});
        DrawTimeline();
        if (plotVisible) {
            ImGui::BeginChild("Plots",{0,0},ImGuiChildFlags_Borders);
            if(ImGui::BeginTabBar("Plot tabs")) {
                if(ImGui::BeginTabItem("Profile",nullptr,plotTab==0?ImGuiTabItemFlags_SetSelected:0)) {
                    if(plotTab==0) plotTab=-1;
                    if(profiles.Enabled()) profiles.DrawPlot({0,0});
                    else ImGui::TextDisabled("Choose 1D profile or 2D profile above.");
                    ImGui::EndTabItem();
                }
                if(ImGui::BeginTabItem("Tracks",nullptr,plotTab==1?ImGuiTabItemFlags_SetSelected:0)) {
                    if(plotTab==1) plotTab=-1;
                    tracking.DrawPlot({0,0},frame);ImGui::EndTabItem();
                }
                if(ImGui::BeginTabItem("Surface",nullptr,plotTab==2?ImGuiTabItemFlags_SetSelected:0)) {
                    if(plotTab==2) plotTab=-1;
                    surfaceTrace.DrawPlot({0,0});ImGui::EndTabItem();
                }
                ImGui::EndTabBar();
            }
            ImGui::EndChild();
        }
        ImGui::EndChild();
        ImGui::End();
    }

    void ReferencePlaneControl() {
        ImGui::Checkbox("Keep reference plane",&keepReferencePlane);
        Tip("Keep the reference grid visible when a mesh is loaded.");
        if(ImGui::TreeNode("Plane orientation")) {
            if(ImGui::Button("XY"))referenceRotation={0,0,0};
            ImGui::SameLine();if(ImGui::Button("XZ"))referenceRotation={-90,0,0};
            ImGui::SameLine();if(ImGui::Button("YZ"))referenceRotation={0,90,0};
            ImGui::TextDisabled("Rotation (degrees)");
            const char* axes[]={"X","Y","Z"};
            float* angles[]={&referenceRotation.x,&referenceRotation.y,&referenceRotation.z};
            for(int i=0;i<3;++i) {
                ImGui::SetNextItemWidth(-30);
                if(ImGui::DragFloat(axes[i],angles[i],.5f,0,0,"%.1f°"))
                    *angles[i]=std::isfinite(*angles[i])?std::remainder(*angles[i],360.0f):0;
            }
            Tip("Rotate about world X, then Y, then Z. Home looks down at the plane from 30° above it.");
            if(ImGui::Button("Align view"))HomeCamera();
            ImGui::TreePop();
        }
    }

    void DrawControls() {
        auto frame=Current();auto* gf=Field(frame);
        if(!frame) {
            ImGui::Spacing();ImGui::TextDisabled("Waiting for frames");
            ImGui::Spacing();ReferencePlaneControl();return;
        }
        ImGui::Spacing();ImGui::TextDisabled("FIELD");
        ImGui::SetNextItemWidth(-1);
        if(ImGui::BeginCombo("##field",fieldName.c_str())) {
            for(const auto& f:frame->fields) if(ImGui::Selectable(f.name.c_str(),f.name==fieldName)) {
                fieldName=f.name;view=0;component=0;automaticColorRange=true;
            }
            ImGui::EndCombo();
        }
        gf=Field(frame);
        ImGui::SetNextItemWidth(-1);
        if(ImGui::Combo("##operation",&view,viewNames,6)) automaticColorRange=true;
        if(gf && (view==2 || view==5)) {
            ImGui::TextDisabled("Component");ImGui::SetNextItemWidth(-1);
            if(ImGui::SliderInt("##component",&component,0,std::max(0,(view==5?(frame->mesh->Dimension()==2?1:3):gf->VectorDim())-1)))
                automaticColorRange=true;
        }
        if(gf) {ImGui::TextDisabled("%s",gf->FESpace()->FEColl()->Name());Tip("Finite element space of the selected field.");}
        if(coloring==0 && gf && ImGui::CollapsingHeader("Color scale")) {
            ImGui::TextDisabled("Color map");ImGui::SetNextItemWidth(-1);
            if(ImGui::BeginCombo("##color_map",RayFEM::ColorMapName(static_cast<RayFEM::ColorMap>(colorMap)))) {
                for(int i=0;i<RayFEM::ColorMapCount;++i)
                    if(ImGui::Selectable(RayFEM::ColorMapName(static_cast<RayFEM::ColorMap>(i)),i==colorMap))colorMap=i;
                ImGui::EndCombo();
            }
            Tip("Viridis, Plasma, Inferno and Cividis are sequential palettes. Coolwarm is diverging; "
                "use symmetric limits to center it at zero. Jet preserves the older rainbow colors.");
            ImGui::Checkbox("Reverse colors",&reverseColors);
            ImGui::TextDisabled("Scale");ImGui::SetNextItemWidth(-1);
            int scaleMode=static_cast<int>(colorScale.scale);
            const char* scales[]={"Linear","Signed log","ZScale"};
            if(ImGui::Combo("##color_scale",&scaleMode,scales,3)) {
                colorScale.scale=static_cast<RayFEM::ColorScale>(scaleMode);
                if(colorScale.scale==RayFEM::ColorScale::ZScale)automaticColorRange=true;
            }
            if(colorScale.scale==RayFEM::ColorScale::SignedLog) {
                bool automaticThreshold=colorScale.log_threshold==0;
                if(ImGui::Checkbox("Automatic log threshold",&automaticThreshold)) {
                    colorScale.log_threshold=0;
                    if(!automaticThreshold) {
                        try {
                            const auto range=automaticColorRange?VolumeRange(frame,*gf):
                                RayFEM::FieldRange{.min_value=colorMinimum,.max_value=colorMaximum};
                            colorScale.log_threshold=RayFEM::ResolveColorScale(range.min_value,range.max_value,colorScale).log_threshold;
                        } catch(const std::exception&) {colorScale.log_threshold=1;}
                    }
                }
                Tip("Signed log is continuous through zero. Automatic threshold is 1% of the largest absolute limit.");
                if(!automaticThreshold) {
                    ImGui::TextDisabled("Near-zero threshold");ImGui::SetNextItemWidth(-1);
                    if(ImGui::InputDouble("##log_threshold",&colorScale.log_threshold,0,0,"%.6g") &&
                        (!std::isfinite(colorScale.log_threshold) || colorScale.log_threshold<=0))
                        colorScale.log_threshold=0;
                }
            } else if(colorScale.scale==RayFEM::ColorScale::ZScale) {
                float contrast=static_cast<float>(colorScale.zscale_contrast);
                ImGui::TextDisabled("ZScale contrast");ImGui::SetNextItemWidth(-1);
                if(ImGui::SliderFloat("##zscale_contrast",&contrast,0,1,"%.2f",ImGuiSliderFlags_AlwaysClamp))
                    colorScale.zscale_contrast=contrast;
                Tip("IRAF / DS9 robust limits from volume samples. Higher contrast narrows the range; zero uses the full range.");
            }
            if(ImGui::Checkbox("Automatic volume range",&automaticColorRange) && !automaticColorRange) {
                try {
                    const auto range=VolumeRange(frame,*gf);
                    const auto mapping=RayFEM::ResolveColorScale(range.min_value,range.max_value,colorScale,range.samples);
                    colorMinimum=mapping.minimum;colorMaximum=mapping.maximum;
                } catch(const std::exception&) {colorMinimum=0;colorMaximum=1;}
            }
            Tip("Sample the complete selected field volume, including parts hidden by clipping. "
                "Material exclusions still apply. Turn off to keep fixed limits across frames.");
            if(automaticColorRange) ImGui::TextDisabled(colorScale.scale==RayFEM::ColorScale::ZScale?
                "ZScale limits · current frame":"Volume samples · current frame");
            else {
                ImGui::TextDisabled("Minimum");ImGui::SetNextItemWidth(-1);
                ImGui::InputDouble("##minimum",&colorMinimum,0,0,"%.6g");
                ImGui::TextDisabled("Maximum");ImGui::SetNextItemWidth(-1);
                ImGui::InputDouble("##maximum",&colorMaximum,0,0,"%.6g");
                if(!std::isfinite(colorMinimum)||!std::isfinite(colorMaximum)||colorMinimum>colorMaximum) {
                    ImGui::PushStyleColor(ImGuiCol_Text,RayFEM::UI::ThemeText(RayFEM::UI::Palette().warning));
                    ImGui::TextWrapped("Use finite limits with minimum ≤ maximum.");
                    ImGui::PopStyleColor();
                }
            }
        }
        ImGui::Spacing();
        if(ImGui::CollapsingHeader("Regions",ImGuiTreeNodeFlags_DefaultOpen)) {
            const char* colors[]={"Field values","Material IDs","Boundary IDs"};
            ImGui::SetNextItemWidth(-1);ImGui::Combo("##coloring",&coloring,colors,3);
            if(ImGui::TreeNode("Materials")) {
                if(ImGui::Button("All")) excludedMaterials.clear();
                ImGui::SameLine();
                if(ImGui::Button("None")) {
                    excludedMaterials.clear();
                    for(int id:frame->mesh->attributes) excludedMaterials.push_back(id);
                }
                const auto actualExclusions = RenderSettings(frame).excluded_materials;
                for(int id:frame->mesh->attributes) {
                    bool included=std::find(excludedMaterials.begin(),excludedMaterials.end(),id)==excludedMaterials.end();
                    const bool unsupported = included &&
                        std::find(actualExclusions.begin(), actualExclusions.end(), id) != actualExclusions.end();
                    ImGui::PushID(id);const auto c=RayFEM::AttributeColor(id);
                    ImGui::BeginDisabled(unsupported);
                    if (unsupported) included = false;
                    if(ImGui::Checkbox(("Material "+std::to_string(id)).c_str(),&included)) {
                        if(included) std::erase(excludedMaterials,id);else excludedMaterials.push_back(id);
                    }
                    ImGui::EndDisabled();
                    if (unsupported && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                        ImGui::SetTooltip("The archive has no %s field on this material.", fieldName.c_str());
                    ImGui::SameLine();
                    ImGui::SetCursorPosY(ImGui::GetCursorPosY()+(ImGui::GetFrameHeight()-12)*.5f);
                    ImGui::ColorButton("##material_color", {c.r/255.f,c.g/255.f,c.b/255.f,1},
                                       ImGuiColorEditFlags_NoTooltip|ImGuiColorEditFlags_NoDragDrop, {12,12});
                    ImGui::PopID();
                }
                std::sort(excludedMaterials.begin(),excludedMaterials.end());
                ImGui::TextDisabled("Applies to rendering and analysis");
                ImGui::TreePop();
            }
        }
        if(ImGui::CollapsingHeader("Appearance",ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Surface",&showSurface);
            ImGui::Checkbox("Element edges",&showBoundaries);
            ImGui::Checkbox("DOF points",&showDofs);
            Tip("Hover for coefficient details. Use Pick DOF to add a time trace.");
            ImGui::TextDisabled("Explode elements");ImGui::SetNextItemWidth(-1);
            ImGui::SliderFloat("##element_explosion",&elementExplosion,0,.95f,"%.2f",ImGuiSliderFlags_AlwaysClamp);
            Tip("Open gaps by shrinking each displayed element around its center. Zero restores the mesh. Field values and analysis use the original domain.");
            ReferencePlaneControl();
            if(ImGui::TreeNode("Detail")) {
                ImGui::Checkbox("Automatic detail",&automaticDetail);
                Tip("Choose surface detail from the mesh and displacement orders.");
                if (automaticDetail)
                    ImGui::TextDisabled("%d subdivisions per edge", RenderSettings(frame).refinement);
                else {
                    ImGui::TextDisabled("Subdivisions");ImGui::SetNextItemWidth(-1);
                    ImGui::SliderInt("##refinement",&refinement,1,32);
                }
                ImGui::Checkbox("Tessellation",&wireframe);
                ImGui::TreePop();
            }
        }
        flow.Controls();
        if(frame->displacement && ImGui::CollapsingHeader("Deformation",ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Apply displacement",&deform);
            ImGui::TextDisabled("Scale");ImGui::SetNextItemWidth(-1);
            ImGui::SliderFloat("##displacement_scale",&displacementScale,-2,3,"%.2f");
        }
        if(ImGui::CollapsingHeader("Clipping")) {
            ImGui::Checkbox("Enable##clipping",&clip);
            const char* modes[]={"Cut elements","Whole elements"};
            ImGui::SetNextItemWidth(-1);ImGui::Combo("##clipmode",&clipMode,modes,2);
            if(clipMode==0) {
                const char* algorithms[]={"Curved / refined","Planar / fast"};
                ImGui::SetNextItemWidth(-1);ImGui::Combo("##cutalgorithm",&cutAlgorithm,algorithms,2);
            }
            ImGui::Checkbox("Plane guide",&planeGuide);
            ImGui::TextDisabled("Position");ImGui::SetNextItemWidth(-1);
            ImGui::DragFloat3("##clip_position",&slicePoint.x,radius*0.01f);
            ImGui::TextDisabled("Normal");ImGui::SetNextItemWidth(-1);
            ImGui::DragFloat3("##clip_normal",&sliceNormal.x,0.01f,-1,1);
        }
        if(!error.empty()) {ImGui::Spacing();ImGui::TextWrapped("%s",error.c_str());}
    }

    std::string TimelineLabel() const {
        auto frame=Current();
        if(!frame) return {};
        char label[96];
        std::snprintf(label,sizeof(label),"Frame %u  ·  t %.5g",frame->metadata.frame_index,frame->metadata.sim_time);
        return label;
    }

    float TimelineHeight(float width) const {
        const auto& style=ImGui::GetStyle();
        const float row=ImGui::GetFrameHeight();
        const float controls=ImGui::CalcTextSize("<").x+ImGui::CalcTextSize(">").x+
            4*style.FramePadding.x+60+100+row+style.ItemInnerSpacing.x+
            ImGui::CalcTextSize("Live").x+4*style.ItemSpacing.x;
        const bool wrap=Current() && controls+style.ItemSpacing.x+ImGui::CalcTextSize(TimelineLabel().c_str()).x>width;
        return row*(Current()?2:1)+style.ItemSpacing.y*3+(wrap?row+style.ItemSpacing.y:0);
    }

    void DrawTimeline() {
        ImGui::Spacing();
        ImGui::BeginDisabled(frames.empty());
        if(ImGui::Button("<")) {if(selected>0)--selected;follow=false;playing=false;}
        Tip("Previous frame");ImGui::SameLine();
        if(ImGui::Button(playing?"Pause":"Play",{60,0})) {playing=!playing;follow=false;lastPlay=GetTime();}
        ImGui::SameLine();
        if(ImGui::Button(">")) {if(selected+1<static_cast<int>(frames.size()))++selected;follow=false;playing=false;}
        Tip("Next frame");ImGui::SameLine();
        ImGui::SetNextItemWidth(100);ImGui::SliderFloat("##fps",&playbackFps,1,60,"%.0f fps");
        Tip("Playback speed");ImGui::SameLine();
        if(ImGui::Checkbox("Live",&follow)) playing=false;
        Tip("Follow incoming frames");
        if(auto frame=Current()) {
            const auto label=TimelineLabel();
            RayFEM::UI::NextInlineItem(ImGui::CalcTextSize(label.c_str()).x);
            ImGui::AlignTextToFramePadding();ImGui::TextDisabled("%s",label.c_str());
            ImGui::SetNextItemWidth(-1);
            if(ImGui::SliderInt("##frame",&selected,0,static_cast<int>(frames.size())-1,"")) {follow=false;playing=false;}
        }
        ImGui::EndDisabled();
    }

    void DrawViewport(ImVec2 size) {
        int width=std::max(1,static_cast<int>(size.x)),height=std::max(1,static_cast<int>(size.y));
        if (!target.id || width!=target.texture.width || height!=target.texture.height) {
            if (target.id) UnloadRenderTexture(target);
            target=LoadRenderTexture(width,height);
        }
        ImVec2 origin=ImGui::GetCursorScreenPos();
        // Use the same immediate input snapshot for hit testing and presses.
        // ImGui can defer a position event when a press arrives in that frame.
        Vector2 mouse=GetMousePosition();
        bool hovered=RayFEM::CameraViewportContains(mouse,{origin.x,origin.y,size.x,size.y}) &&
            IsWindowFocused() &&
            !ImGui::IsPopupOpen(nullptr,ImGuiPopupFlags_AnyPopupId|ImGuiPopupFlags_AnyPopupLevel);
        RayFEM::CameraInputSnapshot input{
            mouse,hovered,
            IsMouseButtonPressed(MOUSE_BUTTON_LEFT),IsMouseButtonDown(MOUSE_BUTTON_LEFT),
            IsMouseButtonPressed(MOUSE_BUTTON_MIDDLE),IsMouseButtonDown(MOUSE_BUTTON_MIDDLE),
            GetMouseWheelMove(),!IsWindowFocused() ||
                ImGui::IsPopupOpen(nullptr,ImGuiPopupFlags_AnyPopupId|ImGuiPopupFlags_AnyPopupLevel)};
        const auto orientation=RayFEM::UI::OrientationControlsBounds({origin.x,origin.y,size.x,size.y});
        const bool existingDrag=cameraDrag.orbiting || cameraDrag.panning || profiles.Dragging() || elementPress;
        const auto cubeInput=RayFEM::UI::UpdateOrientationControlsInput(cubePointer,input,camera,orientation,existingDrag);
        if(cubeInput.selected && RayFEM::BeginCameraViewTransition(cameraTransition,camera,cubeInput.selected->direction,GetTime()))
            cameraAdjusted=true;
        if(cubeInput.home)HomeCamera();
        if(cubeInput.rotation_button>=0) {
            const auto& rotation=RayFEM::UI::CameraRotationButtons[cubeInput.rotation_button];
            if(RayFEM::BeginCameraRotationStep(cameraTransition,camera,rotation.axis,rotation.degrees,GetTime()))
                cameraAdjusted=true;
        }
        const bool meshHovered=hovered && !cubeInput.owns_pointer;
        bool profileDrag=!pickingDof && profiles.HandleInput(origin,size,camera,meshHovered);
        // A short press selects; a drag remains an orbit. Capture the mesh so
        // streamed replacement geometry cannot receive a stale mouse release.
        const auto clickFrame=Current();
        auto meshIdentity=[](const std::shared_ptr<RayFEM::RenderFrame>& frame) {
            return frame?ElementPressIdentity{frame->session_id,frame->mesh_id,frame->mesh_revision,
                frame->mesh?frame->mesh->GetSequence():0}:ElementPressIdentity{};
        };
        if(input.cancel_drag || profileDrag || cubeInput.owns_pointer || pickingDof || input.middle_down ||
           (elementPress && (!clickFrame || elementPressMesh.lock()!=clickFrame->mesh ||
                             elementPressIdentity!=meshIdentity(clickFrame))))elementPress=false;
        if(meshHovered && !profileDrag && !pickingDof && !input.cancel_drag && !input.middle_down && input.left_pressed && clickFrame) {
            elementPress=true;elementPressMoved=false;elementPressPosition=mouse;
            elementPressMesh=clickFrame->mesh;
            elementPressIdentity=meshIdentity(clickFrame);
            elementPressAdditive=RayFEM::UI::MouseSelectionShift();
        }
        if(profileDrag || (meshHovered && (input.left_pressed || input.middle_pressed || input.wheel!=0)))
            cameraTransition.active=false;
        input.viewport_hovered=meshHovered && !profileDrag;
        input.cancel_drag|=cubeInput.owns_pointer || profileDrag || (pickingDof && meshHovered && input.left_down);
        if(elementPress)RayFEM::PrepareCameraOrbitGesture(input,cameraDrag,elementPressPosition,elementPressMoved);
        if(RayFEM::UpdateCameraController(camera,cameraDrag,input,radius))cameraAdjusted=true;
        if (hovered && !ImGui::IsAnyItemActive()) {
            if (IsKeyPressed(KEY_LEFT) && selected>0) {--selected;follow=false;playing=false;}
            if (IsKeyPressed(KEY_RIGHT) && selected+1<static_cast<int>(frames.size())) {++selected;follow=false;playing=false;}
        }
        auto* active=ActiveView();
        if(elementPress && IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
            if(meshHovered && !elementPressMoved && active && active->frame->mesh==elementPressMesh.lock() &&
               elementPressIdentity==meshIdentity(active->frame)) {
                const auto hit=active->picker.Pick(GetScreenToWorldRayEx(
                    {mouse.x-origin.x,mouse.y-origin.y},camera,width,height));
                elements.Select(hit?std::optional<int>{hit->element}:std::nullopt,elementPressAdditive);
                if(hit)inspectorTab=2;
            }
            elementPress=false;
        }
        if(active) {
            try {RefreshSelection(*active);}
            catch(const std::exception& e) {error=e.what();}
        }
        const bool referencePlaneVisible=!active || keepReferencePlane;
        const auto referenceGrid=RayFEM::ReferencePlaneGrid(active?active->bounds:BoundingBox{},radius,
            active && active->vertices,referenceRotation);
        if(active && active->vertices) {profiles.SetBounds(active->bounds,active->frame->mesh);surfaceTrace.SetBounds(active->bounds,active->frame->mesh);}
        if(active) {
            flow.UpdateContext(active->frame,Field(active->frame),fieldName,RenderSettings(active->frame));
            if(active->vertices) flow.SetBounds(active->bounds,active->frame->mesh);
        }
        if (hovered && IsKeyPressed(KEY_R) && active && active->vertices) {cameraAdjusted=false;FitCamera(active->bounds);}
        if(hovered && !ImGui::IsAnyItemActive() && IsKeyPressed(KEY_HOME))HomeCamera();
        if(RayFEM::UpdateCameraViewTransition(camera,cameraTransition,GetTime()))cameraAdjusted=true;
        Vector3 normal=Vector3Normalize(sliceNormal);
        // The CPU cut includes the section cap. A second shader discard would
        // incorrectly remove cap fragments through floating-point roundoff.
        int enabled=0;
        SetShaderValue(shader,normalLoc,&normal,SHADER_UNIFORM_VEC3);
        SetShaderValue(shader,pointLoc,&slicePoint,SHADER_UNIFORM_VEC3);
        SetShaderValue(shader,enabledLoc,&enabled,SHADER_UNIFORM_INT);
        float depthBias=(showBoundaries || wireframe)?1.5f:0;
        SetShaderValue(shader,depthBiasLoc,&depthBias,SHADER_UNIFORM_FLOAT);
        BeginTextureMode(target);
        ClearBackground(RayFEM::UI::Palette().viewport_background);
        const double savedNear=rlGetCullDistanceNear(),savedFar=rlGetCullDistanceFar();
        if (active && active->vertices) {
            // Fit the depth range to the scene. The render texture may have a
            // 16-bit depth buffer; the default .01..1000 range loses enough
            // precision to break coplanar element outlines into dashes.
            auto depthBounds=active->bounds;
            if(referencePlaneVisible) {
                depthBounds.min=Vector3Min(depthBounds.min,referenceGrid.bounds.min);
                depthBounds.max=Vector3Max(depthBounds.max,referenceGrid.bounds.max);
            }
            Vector3 center=Vector3Scale(Vector3Add(depthBounds.min,depthBounds.max),0.5f);
            float extent=std::max(0.01f,Vector3Length(Vector3Subtract(depthBounds.max,depthBounds.min))*0.5f);
            Vector3 forward=Vector3Normalize(Vector3Subtract(camera.target,camera.position));
            float depth=Vector3DotProduct(Vector3Subtract(center,camera.position),forward);
            double nearPlane=std::max(extent*0.001f,depth-extent*2);
            rlSetClipPlanes(nearPlane,std::max(nearPlane*2,static_cast<double>(depth+extent*4)));
        }
        BeginMode3D(camera);
        rlDisableBackfaceCulling();
        if(referencePlaneVisible)DrawReferencePlane(referenceGrid);
        if (active) {
            if (showSurface) DrawModel(active->model,{0,0,0},1,WHITE);
            if(active->selectionModel.meshCount) {
                rlDrawRenderBatchActive();
                if(showSurface)rlDisableDepthMask();
                const float selectionBias=-.25f;
                SetShaderValue(shader,depthBiasLoc,&selectionBias,SHADER_UNIFORM_FLOAT);
                DrawModel(active->selectionModel,{0,0,0},1,WHITE);
                rlDrawRenderBatchActive();
                SetShaderValue(shader,depthBiasLoc,&depthBias,SHADER_UNIFORM_FLOAT);
                rlEnableDepthMask();
            }
            if (wireframe) DrawModelWires(active->model,{0,0,0},1,RayFEM::UI::Palette().mesh_wire);
            if (showBoundaries) DrawBoundaries(*active);
            const RayFEM::ClipPlane flowPlane{slicePoint,sliceNormal};
            flow.Draw(camera,GetTime(),streamlineRenderer,{float(width),float(height)},active->retainedElements.empty()?nullptr:&active->retainedElements,
                      clip && clipMode==0?&flowPlane:nullptr);
        }
        if (planeGuide) DrawPlaneGuide();
        profiles.DrawGuide();
        surfaceTrace.DrawGuide(camera);
        rlEnableBackfaceCulling();
        EndMode3D();
        rlSetClipPlanes(savedNear,savedFar);
        EndTextureMode();
        // Allocate the same viewport item used for camera/handle hit testing,
        // then round only the displayed texture's outer corners.
        ImGui::Dummy(size);
        auto* draw=ImGui::GetWindowDrawList();
        draw->AddImageRounded(ImTextureID(target.texture.id),origin,{origin.x+size.x,origin.y+size.y},
                              {0,0},{1,-1},IM_COL32_WHITE,RayFEM::UI::CardRounding);
        draw->AddRect(origin,{origin.x+size.x,origin.y+size.y},ImGui::GetColorU32(ImGuiCol_Border),
                      RayFEM::UI::CardRounding);
        if (active && showDofs) DrawDofs(*active,origin,size,meshHovered);
        profiles.DrawHandles(origin,size,camera);
        if (active && active->hasFieldRange && coloring==0) {
            const float x=origin.x+24,y=origin.y+size.y-48,w=220;
            draw->AddRectFilled({x-12,y-26},{x+w+12,y+36},RayFEM::UI::Palette().plot_background,RayFEM::UI::OverlayRounding);
            const auto map=static_cast<RayFEM::ColorMap>(colorMap);
            const std::string legend=std::string(RayFEM::ColorScaleName(colorScale.scale))+" · "+RayFEM::ColorMapName(map);
            draw->AddText({x,y-19},RayFEM::UI::Palette().muted,legend.c_str());
            auto color=[&](float t) {
                const auto c=RayFEM::SampleColorMap(map,t,reverseColors);
                return IM_COL32(c.r,c.g,c.b,c.a);
            };
            const double colorMagnitude=std::max(std::abs(active->min),std::abs(active->max));
            if(colorMagnitude==0 || active->max/colorMagnitude-active->min/colorMagnitude<=1e-12)
                draw->AddRectFilled({x,y},{x+w,y+10},color(.5f),5);
            else {
                // The center overlaps the flat edges of the caps, hiding their
                // antialiasing fringe without affecting the rounded exterior.
                for(int end=0;end<2;++end) {
                    const int first=draw->VtxBuffer.Size;
                    const float capX=end?x+w-10:x;
                    draw->AddRectFilled({capX,y},{capX+10,y+10},IM_COL32_WHITE,5,
                        end?ImDrawFlags_RoundCornersRight:ImDrawFlags_RoundCornersLeft);
                    for(int vertex=first;vertex<draw->VtxBuffer.Size;++vertex) {
                        auto& v=draw->VtxBuffer[vertex];
                        v.col=(v.col&IM_COL32_A_MASK)|(color(std::clamp((v.pos.x-x)/w,0.0f,1.0f))&~IM_COL32_A_MASK);
                    }
                }
                for(int i=0;i<32;++i) {
                    const float a=5+(w-10)*i/32,b=5+(w-10)*(i+1)/32;
                    draw->AddRectFilledMultiColor({x+a,y},{x+b,y+10},
                        color(a/w),color(b/w),color(b/w),color(a/w));
                }
            }
            char low[32],high[32];
            std::snprintf(low,sizeof(low),"%.5g",active->min);
            std::snprintf(high,sizeof(high),"%.5g",active->max);
            draw->AddText({x,y+13},RayFEM::UI::Palette().text,low);
            draw->AddText({x+w-ImGui::CalcTextSize(high).x,y+13},RayFEM::UI::Palette().text,high);
            if(active->min<active->max) {
                char middle[32];std::snprintf(middle,sizeof(middle),"%.5g",RayFEM::ColorScaleValue(.5,active->colorMapping));
                const float textWidth=ImGui::CalcTextSize(middle).x;
                if(ImGui::CalcTextSize(low).x+textWidth*.5f+12<w*.5f &&
                   ImGui::CalcTextSize(high).x+textWidth*.5f+12<w*.5f)
                    draw->AddText({x+w*.5f-textWidth*.5f,y+13},RayFEM::UI::Palette().text,middle);
            }
            if(ImGui::IsMouseHoveringRect({x-12,y-26},{x+w+12,y+36})) {
                ImGui::BeginTooltip();
                ImGui::Text("Display limits %.10g to %.10g",active->min,active->max);
                ImGui::Text("Full volume %.10g to %.10g",active->volumeMin,active->volumeMax);
                ImGui::Text("At cursor %.10g",RayFEM::ColorScaleValue((ImGui::GetMousePos().x-x)/w,active->colorMapping));
                if(colorScale.scale==RayFEM::ColorScale::SignedLog)
                    ImGui::Text("Near-zero threshold %.7g",active->colorMapping.log_threshold);
                if(automaticColorRange) {
                    ImGui::Text("%zu volume samples",active->rangeSamples);
                    if(active->rangeBudgetLimited) ImGui::TextUnformatted("Sampling detail reduced to fit the volume budget.");
                    ImGui::TextUnformatted("Includes clipped regions. Hidden elements and materials are excluded.");
                } else ImGui::TextUnformatted("Fixed limits across frames. Change them under Color scale.");
                ImGui::EndTooltip();
            }
        }
        if(active && coloring!=0) {
            const auto& ids=coloring==1?active->frame->mesh->attributes:active->frame->mesh->bdr_attributes;
            const auto actualExclusions = RenderSettings(active->frame).excluded_materials;
            int visibleIds=0;
            for(int id:ids)
                if(coloring!=1 || std::find(actualExclusions.begin(),actualExclusions.end(),id)==actualExclusions.end())
                    ++visibleIds;
            const float legendHeight=std::min(size.y-16,38.0f+22*visibleIds+(coloring==2?24:0));
            draw->AddRectFilled({origin.x+8,origin.y+8},{origin.x+154,origin.y+8+legendHeight},
                                RayFEM::UI::Palette().plot_background,RayFEM::UI::OverlayRounding);
            float y=origin.y+14;
            draw->AddText({origin.x+16,y},RayFEM::UI::Palette().muted,coloring==1?"Material":"Boundary");y+=24;
            for(int id:ids) {
                if(coloring==1 && std::find(actualExclusions.begin(),actualExclusions.end(),id)!=actualExclusions.end()) continue;
                const auto c=RayFEM::AttributeColor(id);
                draw->AddRectFilled({origin.x+16,y+2},{origin.x+27,y+13},IM_COL32(c.r,c.g,c.b,255),2);
                draw->AddText({origin.x+34,y},RayFEM::UI::Palette().text,std::to_string(id).c_str());y+=22;
                if(y>origin.y+size.y-50) break;
            }
            if(coloring==2) draw->AddText({origin.x+16,y+3},RayFEM::UI::Palette().muted,"0: interior / cut");
        }
        if(active && !active->vertices) draw->AddText({origin.x+22,origin.y+size.y/2},RayFEM::UI::Palette().muted,"No visible elements");
        if (!Current()) draw->AddText({origin.x+22,origin.y+22},RayFEM::UI::Palette().muted,"Open a .serif archive or stream a mesh and frame.");
        else if (!active) draw->AddText({origin.x+22,origin.y+22},RayFEM::UI::Palette().warning,"Choose a supported field operation in the left panel.");
        RayFEM::UI::DrawOrientationCube(camera,orientation.cube,mouse,hovered && !existingDrag);
        RayFEM::UI::DrawCameraRotationArrows(orientation,mouse,hovered && !existingDrag,rotationIcons);
    }

    void DrawReferencePlane(const RayFEM::ReferencePlaneLayout& grid) const {
        const auto color=RayFEM::UI::ThemeText(RayFEM::UI::Palette().border);
        const Color lines{static_cast<unsigned char>(color.x*255),static_cast<unsigned char>(color.y*255),
                          static_cast<unsigned char>(color.z*255),255};
        for(int i=0;i<=10;++i) {
            const float offset=grid.half_extent*(i/5.0f-1);
            const auto u=Vector3Scale(grid.u,grid.half_extent),v=Vector3Scale(grid.v,grid.half_extent);
            const auto a=Vector3Add(grid.center,Vector3Scale(grid.u,offset));
            const auto b=Vector3Add(grid.center,Vector3Scale(grid.v,offset));
            DrawLine3D(Vector3Subtract(a,v),Vector3Add(a,v),lines);
            DrawLine3D(Vector3Subtract(b,u),Vector3Add(b,u),lines);
        }
    }

    void DrawBoundaries(ViewCache& active) {
        try {
            if (!active.boundaries_ready) {
                active.boundaries=RayFEM::BuildElementBoundaries(*active.frame->mesh,RenderSettings(active.frame));
                active.boundaries_ready=true;
            }
            if(!active.edges_uploaded) {
                active.edgeModel=edgeRenderer.Upload(active.boundaries);
                active.edges_uploaded=true;
                TrimViewCache();
            }
            edgeRenderer.Draw(active.edgeModel,camera,{float(target.texture.width),float(target.texture.height)},
                radius,RayFEM::UI::Palette().mesh_edges,coloring==2);
        } catch(const std::exception& e){error=e.what();active.boundaries_ready=true;}
    }

    void DrawDofs(ViewCache& active,ImVec2 origin,ImVec2 size,bool hovered) {
        try {
            if (!active.dofs_ready) {
                if (auto* gf=Field(active.frame)) active.dofs=RayFEM::SampleDofs(*active.frame->mesh,*gf,RenderSettings(active.frame),20000,active.retainedElements.empty()?nullptr:&active.retainedElements);
                active.dofs_ready=true;
                TrimViewCache();
            }
        } catch (const std::exception& e) {error=e.what();active.dofs_ready=true;}
        auto* draw=ImGui::GetWindowDrawList();
        draw->PushClipRect(origin,{origin.x+size.x,origin.y+size.y},true);
        const RayFEM::DofPoint* nearest=nullptr;
        float distance=81;
        auto mouse=ImGui::GetMousePos();
        size_t stride=std::max<size_t>(1,(active.dofs.size()+19999)/20000);
        for (size_t i=0;i<active.dofs.size();i+=stride) {
            const auto& point=active.dofs[i];
            if (Vector3DotProduct(Vector3Subtract(point.position,camera.position),Vector3Subtract(camera.target,camera.position))<=0) continue;
            if (clip && clipMode==0 && Vector3DotProduct(Vector3Subtract(point.position,slicePoint),sliceNormal)>0) continue;
            Vector2 p=GetWorldToScreenEx(point.position,camera,target.texture.width,target.texture.height);
            ImVec2 screen={origin.x+p.x,origin.y+p.y};
            const ImU32 trackColor=tracking.DofColor(*active.frame,fieldName,point.index);
            draw->AddCircleFilled(screen,trackColor?5:3,trackColor?trackColor:RayFEM::UI::Palette().dof);
            if(trackColor) draw->AddCircle(screen,7,RayFEM::UI::Palette().dof_ring,0,1.5f);
            float d=(screen.x-mouse.x)*(screen.x-mouse.x)+(screen.y-mouse.y)*(screen.y-mouse.y);
            if (hovered && d<distance) {distance=d;nearest=&point;}
        }
        draw->PopClipRect();
        if (nearest) {
            ImGui::BeginTooltip();
            ImGui::Text("DOF %d | element %d",nearest->index,nearest->element);
            ImGui::Text(elementExplosion>0?"Display position: %.6g, %.6g, %.6g":"Position: %.6g, %.6g, %.6g",nearest->position.x,nearest->position.y,nearest->position.z);
            ImGui::Text("Coefficient: %.9g",nearest->coefficient);
            ImGui::Text("Displayed value: %.9g",nearest->value);
            ImGui::Text("Vector: %.6g, %.6g, %.6g",nearest->vector.x,nearest->vector.y,nearest->vector.z);
            if (nearest->directional) ImGui::TextUnformatted("Directional functional (tangent/normal), not a point value");
            else if (!nearest->interpolatory) ImGui::TextUnformatted("Basis coefficient; marker is a representative position");
            if(pickingDof) ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(RayFEM::UI::Palette().accent),"Click to track this coefficient");
            ImGui::EndTooltip();
            if(pickingDof && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                const int dofComponent=std::clamp(component,0,Field(active.frame)->FESpace()->GetVDim()-1);
                tracking.AddDof(active.frame,fieldName,nearest->index,dofComponent,frames);
                pickingDof=false;inspectorTab=1;plotTab=1;showPlots=true;
            }
        }
    }

    void DrawPlaneGuide() {
        Vector3 n=Vector3Normalize(sliceNormal);
        if (Vector3Length(n)<0.01f) return;
        Vector3 u=Vector3Normalize(Vector3CrossProduct(n,std::abs(n.y)<0.9f?Vector3{0,1,0}:Vector3{1,0,0}));
        Vector3 v=Vector3CrossProduct(n,u);
        Vector3 corners[4];
        for (int i=0;i<4;++i) corners[i]=Vector3Add(slicePoint,Vector3Add(Vector3Scale(u,(i==0||i==3?-1:1)*radius),Vector3Scale(v,(i<2?-1:1)*radius)));
        for (int i=0;i<4;++i) DrawLine3D(corners[i],corners[(i+1)%4],RayFEM::UI::Palette().clip_guide);
        DrawLine3D(slicePoint,Vector3Add(slicePoint,Vector3Scale(n,radius*0.4f)),RayFEM::UI::Palette().clip_guide);
    }

    Options opts;
    RayFEM::Server server;
    ArchiveFileDialog fileDialog;
    std::string archivePath, archiveError;
    std::vector<std::string> archiveWarnings;
    std::vector<std::string> archiveReportWarnings;
    bool archiveReport = false;
    ProfilePanel profiles;
    TrackingPanel tracking;
    SurfaceTracePanel surfaceTrace;
    FlowPanel flow;
    ElementPanel elements;
    bool elementPress=false,elementPressMoved=false,elementPressAdditive=false;
    Vector2 elementPressPosition{};
    std::weak_ptr<mfem::Mesh> elementPressMesh;
    using ElementPressIdentity=std::tuple<std::uint64_t,std::uint64_t,std::uint64_t,long>;
    ElementPressIdentity elementPressIdentity{};
    int coloring=0;
    std::vector<int> excludedMaterials;
    int inspectorTab=-1,plotTab=-1;
    bool showPlots=true,pickingDof=false;
    std::deque<std::shared_ptr<RayFEM::RenderFrame>> frames;
    std::deque<ViewCache> cache;
    std::deque<RangeCache> ranges;
    bool automaticColorRange=true;
    RayFEM::ColorScaleOptions colorScale{};
    int colorMap=static_cast<int>(RayFEM::ColorMap::Viridis);
    bool reverseColors=false;
    double colorMinimum=0,colorMaximum=1;
    int selected=-1,view=0,component=0,refinement=3,clipMode=0,cutAlgorithm=0;
    bool automaticDetail=true;
    std::string fieldName,error,failedKey;
    bool follow=true,playing=false,wireframe=false,showSurface=true,showBoundaries=true,showDofs=false,deform=true,clip=false,planeGuide=false;
    bool cameraAdjusted=false;
    bool keepReferencePlane=false;
    Vector3 referenceRotation{};
    std::array<Texture2D,4> rotationIcons{};
    RayFEM::CameraDragState cameraDrag;
    RayFEM::CameraViewTransition cameraTransition;
    RayFEM::UI::OrientationCubeInputState cubePointer;
    float displacementScale=1,elementExplosion=0,playbackFps=12,radius=1;
    double lastPlay=0;
    std::weak_ptr<mfem::Mesh> fittedMesh;
    Camera3D camera{};
    RenderTexture2D target{};
    Shader shader{};
    RayFEM::UI::EdgeRenderer edgeRenderer;
    RayFEM::UI::StreamlineRenderer streamlineRenderer;
    int normalLoc,pointLoc,enabledLoc,depthBiasLoc;
    Vector3 sliceNormal={1,0,0},slicePoint={0,0,0};
};

int main(int argc,char** argv) {
    Options options;
    CLI::App cli{"RayFEM — stream one mesh and explore many fields and frames"};
    cli.add_option("--listen",options.listen,"Bind address");
    cli.add_option("--port",options.port,"Stream port")->check(CLI::Range(1,65535));
    cli.add_option("--control-port",options.controlPort,"Viewer readiness port (default: stream port + 1)")->check(CLI::Range(1,65535));
    cli.add_option("--fps",options.fps,"Maximum display rate")->check(CLI::Range(1,240));
    cli.add_option("--width",options.width)->check(CLI::Range(960,7680));
    cli.add_option("--height",options.height)->check(CLI::Range(640,4320));
    cli.add_option("--history",options.history,"Maximum retained frames (also bounded by field memory)")->check(CLI::Range(1,10000));
    cli.add_option("file,--open",options.archive,"Open a .serif archive");
    cli.add_flag("--light",options.light,"Start in light mode");
    cli.add_option("--explode",options.explosion,"Element spacing (0 restores the mesh)")->check(CLI::Range(0.0f,.95f));
    CLI11_PARSE(cli,argc,argv);
    try { VisualizerApp app(options);app.Run(); }
    catch (const std::exception& e) { std::cerr<<"RayFEM: "<<e.what()<<'\n';return 1; }
    return 0;
}
