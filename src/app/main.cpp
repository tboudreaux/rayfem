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
#include "extras/IconsFontAwesome6.h"
#include "czmq.h"
#include "libRayfem/network/server.hpp"
#include "libRayfem/mesh/converter.hpp"
#include "libRayfem/mesh/clipping.hpp"
#include "libRayfem/mesh/field_range.hpp"
#include "libRayfem/io/serif.hpp"
#include "file_dialog.hpp"
#include "frame_pipeline.hpp"
#include "indexed_surface.hpp"
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
#include "dof_renderer.hpp"
#include "ray_placement.hpp"
#include "widgets.hpp"
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
    Model dofModel{};
    std::string dofTracks;
    bool dofsUploaded = false;
    RayFEM::UI::ElementPicker picker;
    std::uint64_t selectionRevision=UINT64_MAX;
    std::size_t selectionVertices=0;
    std::string appearance;
    RayFEM::UI::IndexedSurface surface;  // per-vertex values and triangle ranges; GPU holds the rest
    std::size_t gpuVertices = 0;
    std::vector<double> rangeDistribution;
    BoundingBox bounds{};
    double min = 0, max = 0, build_ms = 0;
    size_t vertices = 0;
    size_t rangeSamples = 0;
    bool rangeBudgetLimited = false;
    std::size_t rangeSkippedElements = 0;
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
        flow.SetWorker(&pipeline);
        profiles.SetWorker(&pipeline);
        surfaceTrace.SetWorker(&pipeline);
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
        rlImGuiMergeFontAwesome(14);
        // Section titles use the bold face of the same family when it exists.
        for(const char* path:{"/System/Library/Fonts/Supplemental/Arial Bold.ttf",
                              "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                              "C:/Windows/Fonts/segoeuib.ttf"}) {
            if(std::filesystem::is_regular_file(path)) {
                if((RayFEM::UI::HeadingFont()=ImGui::GetIO().Fonts->AddFontFromFileTTF(path,16,&font))) break;
            }
        }
        RayFEM::UI::ApplyLayout();
        RayFEM::UI::ApplyTheme(opts.light ? RayFEM::UI::ThemeMode::Light : RayFEM::UI::ThemeMode::Dark);
        rotationIcons=RayFEM::UI::LoadRotationIcons();
        shader = LoadShaderFromMemory(sliceVertex, sliceFragment);
        edgeRenderer.Initialize();
        dofRenderer.Initialize();
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
        pipeline.Stop();
        ClearCache();
        flow.ClearGraphics();
        streamlineRenderer.Shutdown();
        edgeRenderer.Shutdown();
        dofRenderer.Shutdown();
        rayPlacement.Shutdown();
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
        // Reconstruction builds MFEM spaces; run it where the stream is decoded,
        // then give the frames their worker copies before the UI uses them.
        auto loaded = pipeline.RunOnWorker([&] {
            auto result = RayFEM::LoadSerifFile(path);
            if (result) pipeline.AttachComputeTwins(result.frames);
            return result;
        });
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
        profiles.SetWorker(&pipeline);
        tracking = TrackingPanel{};
        surfaceTrace = SurfaceTracePanel{};
        surfaceTrace.SetWorker(&pipeline);
        flow = FlowPanel{};
        flow.SetWorker(&pipeline);
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
        for (auto& frame : pipeline.TakeFrames()) {
            tracking.Observe(frame);
            frames.push_back(std::move(frame));
        }
        while (frames.size() > static_cast<size_t>(opts.history) || (frames.size()>1 && HistoryBytes()>256ULL*1024*1024)) {
            // A frame viewed while not following stays put: evict the next
            // oldest instead, so a paused view does not advance on its own.
            const std::size_t victim = !follow && selected == 0 && frames.size() > 1 ? 1 : 0;
            const auto evicted = frames[victim];
            for (auto it=cache.begin(); it!=cache.end();) {
                // The surface on screen (the newest entry) stays until a
                // newer one replaces it; dropping it would blank the view.
                if (it->frame == evicted && &*it != &cache.back()) { ReleaseView(*it); it=cache.erase(it); }
                else ++it;
            }
            frames.erase(frames.begin() + victim);
            if (static_cast<int>(victim) < selected) --selected;
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
        UnloadModel(entry.dofModel);
    }

    void ClearCache() {
        for (auto& entry : cache) ReleaseView(entry);
        cache.clear();
    }

    void TrimViewCache() {
        auto bytes=[](const ViewCache& entry) {
            return entry.gpuVertices*48+entry.vertices*2+entry.boundaries.size()*464+entry.picker.StorageBytes()+entry.selectionVertices*64+
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
            for(int c=0;c<entry.model.meshCount && c<int(entry.surface.chunks.size());++c) {
                auto& mesh=entry.model.meshes[c];
                const auto& values=entry.surface.chunks[c].values;
                for(std::size_t i=0;i<values.size();++i) {
                    const auto color=RayFEM::FieldColor(values[i],mapping,
                        static_cast<RayFEM::ColorMap>(colorMap),reverseColors);
                    mesh.colors[4*i]=color.r;mesh.colors[4*i+1]=color.g;
                    mesh.colors[4*i+2]=color.b;mesh.colors[4*i+3]=color.a;
                }
                UpdateMeshBuffer(mesh,3,mesh.colors,static_cast<int>(values.size()*4),0);
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
        sampling.reduce_to_fit=true;
        auto range=RayFEM::EvaluateFieldRange(*frame->mesh,field,options,sampling);
        ranges.push_back({frame,fieldName,options.view,options.component,options.excluded_materials,options.excluded_elements,range});
        if(ranges.size()>64) ranges.pop_front();
        return range;
    }

    // Finished surfaces from the worker become view-cache entries. Errors only
    // matter for the newest request; older results still fill the cache.
    void ProcessResults() {
        while (auto result = pipeline.TakeResult()) {
            const bool newest = result->request.key == requestedKey;
            if (newest) requestedKey.clear();
            if (!result->error.empty()) {
                if (newest) {error=result->error;failedKey=result->request.key+result->request.appearance;}
                continue;
            }
            // A frame can leave a short history while its surface is built.
            // Still show it when it is newer than the surface on screen.
            const bool retained=std::find(frames.begin(),frames.end(),result->request.frame)!=frames.end();
            const bool newer=cache.empty() || result->request.frame->metadata.frame_index>cache.back().frame->metadata.frame_index ||
                result->request.frame->mesh_revision!=cache.back().frame->mesh_revision;
            if (!retained && !newer) continue;
            try {FinishEntry(*result);}
            catch(const std::exception& e) {error=e.what();}
            if (newest) {error.clear();failedKey.clear();}
        }
    }

    void FinishEntry(RayFEM::UI::SurfaceResult& result) {
        auto& request=result.request;
        auto& data=result.data;
        ViewCache entry;
        entry.frame=request.frame; entry.field=request.field; entry.view=request.view; entry.component=request.component;
        entry.clipping=request.clipping; entry.refinement=request.refinement; entry.displacement=request.displacement;
        entry.appearance=request.appearance;
        entry.bounds=data.bounds; entry.min=data.min_value; entry.max=data.max_value;
        entry.vertices=result.vertex_count;
        entry.boundaries=std::move(result.boundaries); entry.boundaries_ready=result.boundaries_ready;
        entry.retainedElements=std::move(result.retained);
        entry.dofs=std::move(result.dofs); entry.dofs_ready=result.dofs_ready;
        if(result.vertex_count) {
            entry.model=RayFEM::UI::UploadIndexedSurface(result.indexed);
            entry.gpuVertices=result.indexed.vertices;
            entry.picker=std::move(result.picker);
            // The GPU meshes keep positions, normals, colors and indices.
            entry.surface=std::move(result.indexed);
            for(auto& chunk:entry.surface.chunks) {
                std::vector<float>().swap(chunk.positions);std::vector<float>().swap(chunk.normals);
                std::vector<unsigned char>().swap(chunk.colors);std::vector<unsigned short>().swap(chunk.indices);
                if(!result.has_range)std::vector<double>().swap(chunk.values);
            }
        }
        if(result.has_range) {
            entry.colorMapping=result.mapping;
            entry.volumeMin=result.volume_min;entry.volumeMax=result.volume_max;
            entry.rangeSamples=result.range_samples;
            entry.rangeBudgetLimited=result.budget_limited;
            entry.rangeSkippedElements=result.skipped_elements;
            entry.rangeDistribution=std::move(result.distribution);
            entry.hasFieldRange=true;
        }
        for (int i=0;i<entry.model.materialCount;++i) entry.model.materials[i].shader=shader;
        entry.build_ms=result.milliseconds;
        cache.push_back(std::move(entry));
        TrimViewCache();
        const auto& frame=request.frame;
        if (cache.back().vertices && fittedMesh.lock() != frame->mesh) {
            if (!cameraAdjusted) FitCamera(cache.back().bounds,!request.clip);
            fittedMesh=frame->mesh;
        }
    }

    ViewCache* ActiveView() {
        surfaceBuilding=false;
        ProcessResults();
        auto frame = Current();
        elements.Context(frame);
        if (!frame) return nullptr;
        Field(frame);
        const auto settings = RenderSettings(frame);
        const int surfaceRefinement = settings.refinement;
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
        const std::string key = std::to_string(reinterpret_cast<uintptr_t>(frame.get()))+fieldName+":"+std::to_string(view)+":"+std::to_string(component)+":"+std::to_string(surfaceRefinement)+":"+std::to_string(amount)+":"+clipKey;
        if (failedKey==key+appearanceKey) return nullptr;
        if (requestedKey!=key) {
            RayFEM::UI::SurfaceRequest request;
            request.frame=frame; request.key=key; request.appearance=appearanceKey;
            request.field=fieldName; request.clipping=clipKey;
            request.view=view; request.component=component; request.refinement=surfaceRefinement; request.displacement=amount;
            request.options=settings; request.options.displacement=nullptr; request.deform=deform;
            request.clip=clip; request.plane={slicePoint,sliceNormal};
            request.clip_mode=static_cast<RayFEM::ClipMode>(clipMode);
            request.cut_algorithm=static_cast<RayFEM::CutAlgorithm>(cutAlgorithm);
            request.automatic_range=automaticColorRange; request.minimum=colorMinimum; request.maximum=colorMaximum;
            request.boundaries=showBoundaries;
            request.dofs=showDofs;
            pipeline.Request(std::move(request));
            requestedKey=key;
        }
        // Keep showing the last finished surface while this one is built.
        surfaceBuilding=true;
        return cache.empty() ? nullptr : &cache.back();
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
        std::size_t triangles=0;
        for(int id:elements.Selected())triangles+=entry.picker.TrianglesForElement(id).size();
        overlay.positions.reserve(triangles*9);overlay.normals.reserve(triangles*9);
        overlay.colors.reserve(triangles*12);
        for(int id:elements.Selected()) for(auto triangle:entry.picker.TrianglesForElement(id)) {
            const auto [chunk,local]=entry.surface.Locate(triangle);
            const auto& source=entry.model.meshes[chunk];
            for(int corner=0;corner<3;++corner) {
                const std::size_t vertex=source.indices[3*local+corner];
                overlay.positions.insert(overlay.positions.end(),source.vertices+3*vertex,source.vertices+3*vertex+3);
                overlay.normals.insert(overlay.normals.end(),source.normals+3*vertex,source.normals+3*vertex+3);
            }
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

    // Connection state; clicking it pauses or resumes receiving frames.
    void DrawStatusIndicator(const std::string& label) {
        const auto& palette=RayFEM::UI::Palette();
        const bool paused=!server.receiving();
        const ImU32 color=paused||!server.ready()?palette.warning:palette.success;
        ImGui::AlignTextToFramePadding();
        const ImVec2 position=ImGui::GetCursorScreenPos();
        const float radius=ImGui::CalcTextSize(ICON_FA_CIRCLE).x*.3f;
        const float spacing=ImGui::GetStyle().ItemInnerSpacing.x;
        const ImVec2 size{radius*2+spacing+ImGui::CalcTextSize(label.c_str()).x,ImGui::GetFrameHeight()};
        if(ImGui::InvisibleButton("##stream_status",size)) server.setReceiving(paused);
        const bool hovered=ImGui::IsItemHovered();
        auto* draw=ImGui::GetWindowDrawList();
        if(hovered) draw->AddRectFilled({position.x-6,position.y},{position.x+size.x+6,position.y+size.y},RayFEM::UI::WithAlpha(palette.text,16),RayFEM::UI::ControlRounding);
        const float center=position.y+size.y*.5f;
        if(paused) {
            draw->AddRectFilled({position.x,center-radius},{position.x+radius*.7f,center+radius},color);
            draw->AddRectFilled({position.x+radius*1.3f,center-radius},{position.x+radius*2,center+radius},color);
        } else draw->AddCircleFilled({position.x+radius,center},radius,color);
        draw->AddText({position.x+radius*2+spacing,center-ImGui::GetTextLineHeight()*.5f},color,label.c_str());
        if (hovered) {
            if(paused) ImGui::SetTooltip("Not receiving frames; %zu skipped so far. Click to resume.",server.getSkippedFrameCount());
            else ImGui::SetTooltip(server.ready()?"Receiving frames on %s:%d (%zu so far). Click to pause.":"Starting the stream listener on %s:%d (%zu frames).",
                                   opts.listen.c_str(),opts.port,server.getReceivedFrameCount());
        }
    }

    void DrawUI() {
        ImGui::SetNextWindowPos({0,0});
        ImGui::SetNextWindowSize({static_cast<float>(GetScreenWidth()),static_cast<float>(GetScreenHeight())});
        ImGui::Begin("RayFEM",nullptr,ImGuiWindowFlags_NoDecoration|ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoSavedSettings);
        const bool light = RayFEM::UI::CurrentTheme() == RayFEM::UI::ThemeMode::Light;
        const char* openLabel=ICON_FA_FOLDER_OPEN "  Open";
        const char* themeLabel=light?ICON_FA_MOON "###theme":ICON_FA_SUN "###theme";
        const char* infoLabel=ICON_FA_CIRCLE_INFO "###info";
        const std::string statusLabel=!server.receiving()?"Paused · "+std::to_string(server.getSkippedFrameCount())+" skipped":
            server.ready()?"Listening on :"+std::to_string(opts.port):"Starting listener";
        const auto& style=ImGui::GetStyle();
        auto buttonWidth=[&](const char* label) {return ImGui::CalcTextSize(label,nullptr,true).x+style.FramePadding.x*2;};
        const float statusWidth=ImGui::CalcTextSize(ICON_FA_CIRCLE).x*.6f+style.ItemInnerSpacing.x+ImGui::CalcTextSize(statusLabel.c_str()).x;
        const float actionsWidth=buttonWidth(openLabel)+buttonWidth(themeLabel)+buttonWidth(infoLabel)+
            statusWidth+3*style.ItemSpacing.x;
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
        if (ImGui::Button(openLabel)) fileDialog.Open();
        Tip("Open a .serif archive (Ctrl/Cmd+O). Files can also be dropped into the window.");
        ImGui::SameLine();
        if (ImGui::Button(themeLabel))
            RayFEM::UI::ApplyTheme(light ? RayFEM::UI::ThemeMode::Dark : RayFEM::UI::ThemeMode::Light);
        Tip(light?"Switch to dark mode":"Switch to light mode");
        ImGui::SameLine();
        DrawStatusIndicator(statusLabel);
        ImGui::SameLine();
        if (ImGui::Button(infoLabel)) ImGui::OpenPopup("Session info");
        Tip("Session details, statistics and mouse controls");
        if (ImGui::BeginPopup("Session info")) {
            ImGui::SeparatorText("Stream");
            bool receiving=server.receiving();
            if(RayFEM::UI::Switch("Receive frames",&receiving)) server.setReceiving(receiving);
            if(ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))
                ImGui::SetTooltip("Turn off to stop taking new frames, for example to inspect the current history. Mesh registrations are still kept.");
            if(!receiving) ImGui::TextDisabled("%zu frames skipped while paused",server.getSkippedFrameCount());
            ImGui::Text("%s:%d",opts.listen.c_str(),opts.port);
            ImGui::Text("%zu received / %zu queued",server.getReceivedFrameCount(),server.getPendingFrameCount());
            ImGui::Text("%zu dropped / %zu errors",server.getDroppedFrameCount(),server.getErrorCount());
            ImGui::Text("%zu retained frames / ~%.1f MiB",frames.size(),HistoryBytes()/1048576.0);
            ImGui::Text("%d fps",GetFPS());
            if (!cache.empty()) ImGui::Text("%zu triangles / %.1f ms build",cache.back().vertices/3,cache.back().build_ms);
            if (!server.getLastError().empty()) ImGui::TextWrapped("%s",server.getLastError().c_str());
            ImGui::Text("SERiF files: %s", RayFEM::SerifLoadingAvailable()?"available":"not enabled in this build");
            if (!archivePath.empty()) {
                ImGui::SeparatorText("Archive");
                ImGui::TextWrapped("%s", archivePath.c_str());
                for (const auto& warning : archiveWarnings) ImGui::TextWrapped("%s", warning.c_str());
            }
            ImGui::Separator();
            ImGui::SeparatorText("Controls");
            if (ImGui::BeginTable("##controls",2,ImGuiTableFlags_SizingFixedFit)) {
                const char* controls[][2]={
                    {"Click","Select an element"},{"Shift-click","Add or remove from the selection"},
                    {"Drag","Orbit"},{"Middle-drag","Pan"},{"Scroll","Zoom"},
                    {"R","Fit the mesh in view"},{"Home","Return to the home view"},
                    {"Left / Right","Step one frame"},{"Ctrl/Cmd+O","Open an archive"},
                    {"Esc","Cancel DOF picking"},
                    {"Orientation cube","Click a face, edge or corner to align"},
                    {"Cube arrows","Rotate 5° (single) or 45° (double)"}};
                for (const auto& [input,effect] : controls) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);ImGui::TextUnformatted(input);
                    ImGui::TableSetColumnIndex(1);ImGui::TextDisabled("%s",effect);
                }
                ImGui::EndTable();
            }
            ImGui::EndPopup();
        }
        if ((ImGui::GetIO().KeyCtrl || ImGui::GetIO().KeySuper) && ImGui::IsKeyPressed(ImGuiKey_O))
            fileDialog.Open();
        DrawArchiveDialogs();
        ImGui::Spacing();
        auto frame=Current();auto* gf=Field(frame);
        elements.Context(frame);
        profiles.UpdateContext(frame,gf,fieldName,RenderSettings(frame));
        if(const auto request=profiles.TakePlacementRequest()) {
            using Kind=RayFEM::UI::RayPlacement::Kind;
            if(*request==ProfilePanel::PlaneCenterRequest) {
                if(profiles.CenterAnchor().Valid())rayPlacement.Edit(Kind::Point,0,{profiles.CenterAnchor()});
                else rayPlacement.Begin(Kind::Point);
            } else if(*request<0)rayPlacement.Begin();
            else rayPlacement.Edit(*request,profiles.Anchors());
        }
        SyncProfileClipping();
        if(const auto request=surfaceTrace.TakePlacementRequest()) {
            using Kind=RayFEM::UI::RayPlacement::Kind;
            if(request->kind==Kind::Path && request->index>=0)rayPlacement.Edit(Kind::Path,request->index,surfaceTrace.PathPoints());
            else rayPlacement.Begin(request->kind);
        }
        surfaceTrace.UpdateContext(frame,gf,fieldName,RenderSettings(frame));
        flow.UpdateContext(frame,gf,fieldName,RenderSettings(frame));
        tracking.SetExcludedMaterials(excludedMaterials,frames);
        tracking.SetExcludedElements(elements.Hidden(),frame,frames);
        tracking.ProcessPending();
        // Drag the inspector's right edge to widen it; the size lasts for the session.
        ImGui::SetNextWindowSizeConstraints({260,0},{std::max(260.0f,GetScreenWidth()*.45f),FLT_MAX});
        ImGui::BeginChild("Inspector",{340,0},ImGuiChildFlags_Borders|ImGuiChildFlags_ResizeX);
        ImGui::PushItemWidth(164);
        // Page switcher, fixed above the scrolling page.
        if(inspectorTab>=0) {inspectorPage=inspectorTab;inspectorTab=-1;}
        RayFEM::UI::Segmented("##inspector_pages",&inspectorPage,
            {ICON_FA_EYE " View",ICON_FA_MAGNIFYING_GLASS_CHART " Analyze",ICON_FA_CUBES " Elements"});
        if(frame) {
            char summary[160];
            std::snprintf(summary,sizeof(summary),"%d elements  ·  %dD  ·  %zu fields  ·  frame %u",
                          frame->mesh->GetNE(),frame->mesh->Dimension(),frame->fields.size(),frame->metadata.frame_index);
            ImGui::TextDisabled("%s",summary);
        }
        ImGui::Spacing();
        // A small gutter keeps cards off the scrollbar and the inspector edge.
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding,{6,0});
        ImGui::BeginChild("Inspector page",{0,0},ImGuiChildFlags_AlwaysUseWindowPadding);
        ImGui::PopStyleVar();
        switch(inspectorPage) {
        case 0: DrawControls(); break;
        case 1:
            profiles.Controls(frame,gf,fieldName,RenderSettings(frame));
            surfaceTrace.Controls(frame,gf,fieldName,RenderSettings(frame));
            tracking.Controls(frame,fieldName,RenderSettings(frame),frames);
            break;
        default: elements.Controls(frame,gf,fieldName,RenderSettings(frame)); break;
        }
        ImGui::EndChild();
        if(tracking.TakePlotRequest()) {plotTab=1;showPlots=true;}
        ImGui::PopItemWidth();
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginChild("Workspace",{0,0},ImGuiChildFlags_AlwaysUseWindowPadding);
        // Toolbar groups: camera, picking, analysis views, then panels. Lit
        // buttons show which of these modes are currently on.
        auto action = [](const char* label, bool active, const char* tip, bool group = false) {
            const auto& style = ImGui::GetStyle();
            const float gap = group ? style.ItemSpacing.x*2 : 0;
            const float width = ImGui::CalcTextSize(label, nullptr, true).x + style.FramePadding.x*2;
            if (RayFEM::UI::NextInlineItem(width + gap) && group) {
                const float x = ImGui::GetItemRectMax().x + style.ItemSpacing.x*1.5f;
                const ImVec2 min = ImGui::GetItemRectMin(), max = ImGui::GetItemRectMax();
                ImGui::GetWindowDrawList()->AddLine({x, min.y+5}, {x, max.y-5}, ImGui::GetColorU32(ImGuiCol_Border));
                ImGui::SameLine(0, style.ItemSpacing.x + gap);
            }
            const bool pressed = RayFEM::UI::ToggleButton(label, active);
            Tip(tip);
            return pressed;
        };
        if(ImGui::Button(ICON_FA_EXPAND "  Fit")) {cameraAdjusted=false;if(auto* active=ActiveView();active && active->vertices) FitCamera(active->bounds);}
        Tip("Fit mesh in view (R)");
        if(action(pickingDof?ICON_FA_XMARK "  Cancel pick###pick":ICON_FA_CROSSHAIRS "  Pick DOF###pick",pickingDof,
                  pickingDof?"Stop picking (Esc)":"Pick a degree of freedom in the mesh to track its coefficient over time.",true)) {
            pickingDof=!pickingDof;if(pickingDof) showDofs=true;
        }
        if(action(ICON_FA_RULER_HORIZONTAL "  1D profile",profiles.Mode()==1,"Sample the field along a ray. Drag the handles in the viewport.",true))
            {profiles.OpenRay();surfaceTrace.SetEnabled(false);inspectorTab=1;plotTab=0;showPlots=true;}
        if(action(ICON_FA_TABLE_CELLS "  2D profile",profiles.Mode()==2,"Sample the field across a plane as a heatmap."))
            {profiles.OpenPlane();surfaceTrace.SetEnabled(false);inspectorTab=1;plotTab=0;showPlots=true;}
        if(action(ICON_FA_GLOBE "  Surface trace",surfaceTrace.Enabled(),"Sample the field along a meridian or latitude of the outer surface."))
            {surfaceTrace.Open();profiles.SetMode(0);inspectorTab=1;plotTab=2;showPlots=true;}
        if(action(ICON_FA_CHART_LINE "  Tracks",false,"Show values tracked across frames."))
            {inspectorTab=1;plotTab=1;showPlots=true;}
        if(action(ICON_FA_WIND "  Flow",flow.Enabled(),"Show arrows or streamlines of a vector field."))
            {flow.Open();inspectorTab=0;}
        if(action(ICON_FA_SQUARE_POLL_VERTICAL "  Plots",showPlots,showPlots?"Hide the plot panel":"Show the plot panel",true))
            showPlots=!showPlots;
        const float available=ImGui::GetContentRegionAvail().y;
        const bool plotVisible=showPlots && (profiles.Enabled() || surfaceTrace.Enabled() || tracking.HasTracks() || plotTab==1);
        float plotHeight=0;
        if(plotVisible) {
            const float maximum=std::max(140.0f,available-TimelineHeight(ImGui::GetContentRegionAvail().x)-200);
            plotHeight=std::clamp(plotPanelHeight>0?plotPanelHeight:std::clamp(available*0.40f,200.0f,320.0f),
                                  std::min(140.0f,maximum),maximum);
        }
        const float timelineHeight=TimelineHeight(ImGui::GetContentRegionAvail().x);
        DrawViewport({ImGui::GetContentRegionAvail().x,std::max(120.0f,available-timelineHeight-(plotVisible?plotHeight+style.ItemSpacing.y:0))});
        DrawTimeline();
        if (plotVisible) {
            PlotSplitter(plotHeight);
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
        DrawRayPlacement();
    }

    // While the 2D profile cuts the mesh, the clipping plane follows it; the
    // user's own clipping settings come back when it stops.
    void SyncProfileClipping() {
        if(profiles.ClipsMesh()) {
            if(!profileClipping){savedClip=clip;savedSlicePoint=slicePoint;savedSliceNormal=sliceNormal;profileClipping=true;}
            const Vector3 n=profiles.PlaneNormal();
            if(std::isfinite(n.x) && Vector3Length(n)>0){clip=true;slicePoint=profiles.PlaneCenter();sliceNormal=n;}
        } else if(profileClipping) {
            clip=savedClip;slicePoint=savedSlicePoint;sliceNormal=savedSliceNormal;profileClipping=false;
        }
    }

    // The element viewer for ray endpoints, as its own window over the viewport.
    void DrawRayPlacement() {
        if(!rayPlacement.Active())return;
        const auto frame=Current();
        if(!frame){rayPlacement.Cancel();return;}
        const auto settings=RenderSettings(frame);
        const ViewCache* shown=cache.empty()?nullptr:&cache.back();
        auto color=[&](double value) {
            if(shown && shown->hasFieldRange)
                return RayFEM::FieldColor(value,shown->colorMapping,static_cast<RayFEM::ColorMap>(colorMap),reverseColors);
            return Color{155,195,220,255};
        };
        // Faint whole-mesh edges (or a see-through surface when edges are off)
        // show where the element sits. The exploded view would not line up.
        auto context=[&](const Camera3D& view,Vector2 viewport) {
            if(!shown || elementExplosion>0) return;
            const auto& palette=RayFEM::UI::Palette();
            const ImU32 text=palette.text;
            const Color faint{static_cast<unsigned char>(text&0xff),static_cast<unsigned char>((text>>8)&0xff),
                              static_cast<unsigned char>((text>>16)&0xff),70};
            if(shown->edges_uploaded && shown->edgeModel.meshCount) {
                edgeRenderer.Draw(shown->edgeModel,view,viewport,radius,faint,false);
            } else if(shown->model.meshCount) {
                rlDisableDepthMask();
                DrawModel(shown->model,{0,0,0},1,{255,255,255,28});
                rlDrawRenderBatchActive();
                rlEnableDepthMask();
            }
        };
        if(const auto result=rayPlacement.Draw(frame,coloring==0?Field(frame):nullptr,settings,std::max(4,settings.refinement),
                                               viewportOrigin,viewportSize,shader,color,camera,context)) {
            if(result->kind==RayFEM::UI::RayPlacement::Kind::Ray) {
                if(result->anchors.size()==2) profiles.SetAnchors({result->anchors[0],result->anchors[1]});
                if(!profiles.Enabled())profiles.OpenRay();
                plotTab=0;
            } else if(result->kind==RayFEM::UI::RayPlacement::Kind::Point) {
                if(!result->anchors.empty())profiles.SetCenterAnchor(result->anchors.front());
                plotTab=0;
            } else {
                surfaceTrace.ApplyPlacement(*result);
                if(!result->cancelled){profiles.SetMode(0);plotTab=2;}
            }
            inspectorTab=1;showPlots=true;
        }
    }

    // A horizontal grip above the plots; dragging it trades viewport and plot height.
    void PlotSplitter(float plotHeight) {
        const float width=ImGui::GetContentRegionAvail().x;
        ImGui::InvisibleButton("##plot_splitter",{width,8});
        const bool hovered=ImGui::IsItemHovered(),active=ImGui::IsItemActive();
        if(hovered || active) ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
        if(active && ImGui::GetIO().MouseDelta.y!=0) plotPanelHeight=plotHeight-ImGui::GetIO().MouseDelta.y;
        if(hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) plotPanelHeight=0;
        Tip("Drag to resize the plots. Double-click to restore the default height.");
        const ImVec2 min=ImGui::GetItemRectMin(),max=ImGui::GetItemRectMax();
        const float center=(min.x+max.x)*.5f,y=(min.y+max.y)*.5f;
        const ImU32 color=hovered || active?RayFEM::UI::Palette().accent:ImGui::GetColorU32(ImGuiCol_Border);
        ImGui::GetWindowDrawList()->AddLine({center-24,y},{center+24,y},color,3);
    }

    void ReferencePlaneControl() {
        RayFEM::UI::Switch("Keep reference plane",&keepReferencePlane);
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
            ImGui::TextDisabled("Waiting for frames");
            if(RayFEM::UI::BeginSection("Reference plane")){ReferencePlaneControl();RayFEM::UI::EndSection();}
            return;
        }
        if(RayFEM::UI::BeginSection("Field")) {
        RayFEM::UI::FieldLabel("Field");
        if(ImGui::BeginCombo("##field",fieldName.c_str())) {
            for(const auto& f:frame->fields) if(ImGui::Selectable(f.name.c_str(),f.name==fieldName)) {
                fieldName=f.name;view=0;component=0;automaticColorRange=true;
            }
            ImGui::EndCombo();
        }
        gf=Field(frame);
        RayFEM::UI::FieldLabel("Show");
        if(ImGui::Combo("##operation",&view,viewNames,6)) automaticColorRange=true;
        if(gf && (view==2 || view==5)) {
            RayFEM::UI::FieldLabel("Component");
            if(ImGui::SliderInt("##component",&component,0,std::max(0,(view==5?(frame->mesh->Dimension()==2?1:3):gf->VectorDim())-1)))
                automaticColorRange=true;
        }
        if(gf) {
            RayFEM::UI::FieldLabel("Space");ImGui::TextDisabled("%s",gf->FESpace()->FEColl()->Name());
            Tip("Finite element space of the selected field.");
        }
        RayFEM::UI::EndSection();
        }
        bool colorScaleOpen=false;
        if(coloring==0 && gf) {
            std::string badge=colorScale.scale==RayFEM::ColorScale::Linear?"":RayFEM::ColorScaleName(colorScale.scale);
            if(!automaticColorRange) badge+=badge.empty()?"Fixed":" · Fixed";
            colorScaleOpen=RayFEM::UI::BeginSection("Color",badge.c_str());
        }
        if(colorScaleOpen) {
            ImGui::SetNextItemWidth(-1);
            RayFEM::UI::ColorMapPicker("##color_map",&colorMap,reverseColors);
            Tip("Viridis, Plasma, Inferno and Cividis are sequential palettes. Coolwarm is diverging; "
                "use symmetric limits to center it at zero. Jet preserves the older rainbow colors.");
            RayFEM::UI::Switch("Reverse colors",&reverseColors);
            int scaleMode=static_cast<int>(colorScale.scale);
            if(RayFEM::UI::Segmented("##color_scale",&scaleMode,{"Linear","Signed log","ZScale"})) {
                colorScale.scale=static_cast<RayFEM::ColorScale>(scaleMode);
                if(colorScale.scale==RayFEM::ColorScale::ZScale)automaticColorRange=true;
            }
            if(colorScale.scale==RayFEM::ColorScale::SignedLog) {
                bool automaticThreshold=colorScale.log_threshold==0;
                if(RayFEM::UI::Switch("Automatic log threshold",&automaticThreshold)) {
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
                    RayFEM::UI::FieldLabel("Near-zero threshold");
                    if(ImGui::InputDouble("##log_threshold",&colorScale.log_threshold,0,0,"%.6g") &&
                        (!std::isfinite(colorScale.log_threshold) || colorScale.log_threshold<=0))
                        colorScale.log_threshold=0;
                }
            } else if(colorScale.scale==RayFEM::ColorScale::ZScale) {
                float contrast=static_cast<float>(colorScale.zscale_contrast);
                RayFEM::UI::FieldLabel("ZScale contrast");
                if(ImGui::SliderFloat("##zscale_contrast",&contrast,0,1,"%.2f",ImGuiSliderFlags_AlwaysClamp))
                    colorScale.zscale_contrast=contrast;
                Tip("IRAF / DS9 robust limits from volume samples. Higher contrast narrows the range; zero uses the full range.");
            }
            if(RayFEM::UI::Switch("Automatic volume range",&automaticColorRange) && !automaticColorRange) {
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
                RayFEM::UI::FieldLabel("Minimum");
                ImGui::InputDouble("##minimum",&colorMinimum,0,0,"%.6g");
                RayFEM::UI::FieldLabel("Maximum");
                ImGui::InputDouble("##maximum",&colorMaximum,0,0,"%.6g");
                if(!std::isfinite(colorMinimum)||!std::isfinite(colorMaximum)||colorMinimum>colorMaximum) {
                    ImGui::PushStyleColor(ImGuiCol_Text,RayFEM::UI::ThemeText(RayFEM::UI::Palette().warning));
                    ImGui::TextWrapped("Use finite limits with minimum ≤ maximum.");
                    ImGui::PopStyleColor();
                }
            }
            RayFEM::UI::EndSection();
        }
        const std::string hiddenMaterials=excludedMaterials.empty()?"":std::to_string(excludedMaterials.size())+" hidden";
        if(RayFEM::UI::BeginSection("Regions",hiddenMaterials.c_str())) {
            ImGui::TextDisabled("Color by");
            RayFEM::UI::Segmented("##coloring",&coloring,{"Field","Material","Boundary"});
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
                    if(RayFEM::UI::Switch(("Material "+std::to_string(id)).c_str(),&included)) {
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
            RayFEM::UI::EndSection();
        }
        if(RayFEM::UI::BeginSection("Display")) {
            RayFEM::UI::Switch("Surface",&showSurface);
            RayFEM::UI::Switch("Element edges",&showBoundaries);
            RayFEM::UI::Switch("DOF points",&showDofs);
            Tip("Hover for coefficient details. Use Pick DOF to add a time trace.");
            RayFEM::UI::FieldLabel("Explode elements");
            ImGui::SliderFloat("##element_explosion",&elementExplosion,0,.95f,"%.2f",ImGuiSliderFlags_AlwaysClamp);
            Tip("Open gaps by shrinking each displayed element around its center. Zero restores the mesh. Field values and analysis use the original domain.");
            ReferencePlaneControl();
            if(ImGui::TreeNode("Detail")) {
                RayFEM::UI::Switch("Automatic detail",&automaticDetail);
                Tip("Choose surface detail from the mesh and displacement orders.");
                if (automaticDetail)
                    ImGui::TextDisabled("%d subdivisions per edge", RenderSettings(frame).refinement);
                else {
                    RayFEM::UI::FieldLabel("Subdivisions");
                    ImGui::SliderInt("##refinement",&refinement,1,32);
                }
                RayFEM::UI::Switch("Tessellation",&wireframe);
                ImGui::TreePop();
            }
            RayFEM::UI::EndSection();
        }
        if(frame->displacement) {
            char badge[32]="Off";
            if(deform && displacementScale!=1) std::snprintf(badge,sizeof(badge),"×%.2f",displacementScale);
            if(RayFEM::UI::BeginSection("Deformation",deform && displacementScale==1?nullptr:badge)) {
                RayFEM::UI::Switch("Apply displacement",&deform);
                RayFEM::UI::FieldLabel("Scale");
                ImGui::SliderFloat("##displacement_scale",&displacementScale,-2,3,"%.2f");
                RayFEM::UI::EndSection();
            }
        }
        if(RayFEM::UI::BeginSection("Clipping",clip?"On":nullptr,false)) {
            if(profileClipping)
                ImGui::TextColored(RayFEM::UI::ThemeText(RayFEM::UI::Palette().accent),ICON_FA_LINK "  Following the 2D profile plane");
            if(profileClipping && ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))
                ImGui::SetTooltip("Turn off Cut the mesh here in the 2D profile to edit clipping again.");
            ImGui::BeginDisabled(profileClipping);
            RayFEM::UI::Switch("Enable##clipping",&clip);
            RayFEM::UI::Segmented("##clipmode",&clipMode,{"Cut elements","Whole elements"});
            if(clipMode==0)
                RayFEM::UI::Segmented("##cutalgorithm",&cutAlgorithm,{"Curved","Planar"});
            if(clipMode==0 && ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))
                ImGui::SetTooltip("Curved follows high-order geometry through each cut cell; Planar is faster.");
            RayFEM::UI::Switch("Plane guide",&planeGuide);
            ImGui::TextDisabled("Position");ImGui::SetNextItemWidth(-1);
            ImGui::DragFloat3("##clip_position",&slicePoint.x,radius*0.01f);
            ImGui::TextDisabled("Normal");ImGui::SetNextItemWidth(-1);
            ImGui::DragFloat3("##clip_normal",&sliceNormal.x,0.01f,-1,1);
            {
                const float presetWidth=(ImGui::GetContentRegionAvail().x-4*ImGui::GetStyle().ItemSpacing.x)/5;
                const Vector3 axes[]={{1,0,0},{0,1,0},{0,0,1}};
                const char* axisLabels[]={"X","Y","Z"};
                for(int i=0;i<3;++i) {
                    if(i)ImGui::SameLine();
                    if(ImGui::Button(axisLabels[i],{presetWidth,0}))sliceNormal=axes[i];
                    if(ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))ImGui::SetTooltip("Normal along +%s",axisLabels[i]);
                }
                ImGui::SameLine();
                if(ImGui::Button(ICON_FA_VIDEO "###clip_view",{presetWidth,0}))
                    sliceNormal=Vector3Normalize(Vector3Subtract(camera.position,camera.target));
                Tip("Face the camera: cut away the half nearer the viewer.");
                ImGui::SameLine();
                if(ImGui::Button(ICON_FA_RIGHT_LEFT "###clip_flip",{presetWidth,0}))sliceNormal=Vector3Negate(sliceNormal);
                Tip("Flip the normal to keep the other side.");
                if(ImGui::Button(ICON_FA_CROSSHAIRS "  Through view center",{-1,0}))slicePoint=camera.target;
                Tip("Move the plane through the point the camera orbits around.");
            }
            ImGui::EndDisabled();
            RayFEM::UI::EndSection();
        }
        flow.Controls();
    }

    static constexpr const char* previousFrameLabel=ICON_FA_BACKWARD_STEP "###previous";
    static constexpr const char* nextFrameLabel=ICON_FA_FORWARD_STEP "###next";
    static constexpr const char* liveLabel=ICON_FA_TOWER_BROADCAST "  Live###live";
    static constexpr float playButtonWidth=92;

    std::string TimelineLabel() const {
        auto frame=Current();
        if(!frame) return {};
        char label[96];
        std::snprintf(label,sizeof(label),"Frame %u  ·  t %.5g",frame->metadata.frame_index,frame->metadata.sim_time);
        return label;
    }

    static constexpr float timelinePadding=10;

    // Height of the timeline card, including the gap above it.
    float TimelineHeight(float width) const {
        const auto& style=ImGui::GetStyle();
        const float row=ImGui::GetFrameHeight();
        const bool wrap=Current() && TimelineControlsWidth()+style.ItemSpacing.x*2+
            ImGui::CalcTextSize(TimelineLabel().c_str()).x>width-2*timelinePadding;
        return style.ItemSpacing.y+2*timelinePadding+row*(Current()?2:1)+(Current()?style.ItemSpacing.y:0)+(wrap?row+style.ItemSpacing.y:0);
    }

    float TimelineControlsWidth() const {
        const auto& style=ImGui::GetStyle();
        auto button=[&](const char* label) {return ImGui::CalcTextSize(label,nullptr,true).x+2*style.FramePadding.x;};
        return button(previousFrameLabel)+button(nextFrameLabel)+playButtonWidth+12+100+button(liveLabel)+3*style.ItemSpacing.x;
    }

    // The timeline as a card: a transport group, speed and Live, the current
    // frame, then a scrubber across all retained frames.
    void DrawTimeline() {
        const auto& style=ImGui::GetStyle();
        const auto& palette=RayFEM::UI::Palette();
        const bool light=RayFEM::UI::CurrentTheme()==RayFEM::UI::ThemeMode::Light;
        const float height=TimelineHeight(ImGui::GetContentRegionAvail().x)-style.ItemSpacing.y;
        ImGui::PushStyleColor(ImGuiCol_ChildBg,light?IM_COL32(20,45,70,10):IM_COL32(255,255,255,9));
        ImGui::PushStyleColor(ImGuiCol_Border,RayFEM::UI::WithAlpha(palette.border,light?120:90));
        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding,RayFEM::UI::CardRounding);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding,{timelinePadding+2,timelinePadding});
        ImGui::BeginChild("Timeline",{0,height},ImGuiChildFlags_Borders|ImGuiChildFlags_AlwaysUseWindowPadding,
                          ImGuiWindowFlags_NoScrollbar|ImGuiWindowFlags_NoScrollWithMouse);
        ImGui::BeginDisabled(frames.empty());
        // Transport buttons share one pill.
        const float row=ImGui::GetFrameHeight();
        auto button=[&](const char* label) {return ImGui::CalcTextSize(label,nullptr,true).x+2*style.FramePadding.x;};
        const float groupWidth=button(previousFrameLabel)+playButtonWidth+button(nextFrameLabel)+4+8;
        const ImVec2 group=ImGui::GetCursorScreenPos();
        ImGui::GetWindowDrawList()->AddRectFilled(group,{group.x+groupWidth,group.y+row},ImGui::GetColorU32(ImGuiCol_FrameBg),RayFEM::UI::ControlRounding);
        ImGui::PushStyleColor(ImGuiCol_Button,IM_COL32(0,0,0,0));
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing,{2,style.ItemSpacing.y});
        ImGui::SetCursorScreenPos({group.x+4,group.y});
        if(ImGui::Button(previousFrameLabel)) {if(selected>0)--selected;follow=false;playing=false;}
        Tip("Previous frame (Left arrow over the viewport)");ImGui::SameLine();
        if(RayFEM::UI::ToggleButton(playing?ICON_FA_PAUSE "  Pause###play":ICON_FA_PLAY "  Play###play",playing,{playButtonWidth,0}))
            {playing=!playing;follow=false;lastPlay=GetTime();}
        Tip(playing?"Pause playback":"Play retained frames in a loop");
        ImGui::SameLine();
        if(ImGui::Button(nextFrameLabel)) {if(selected+1<static_cast<int>(frames.size()))++selected;follow=false;playing=false;}
        Tip("Next frame (Right arrow over the viewport)");
        ImGui::PopStyleVar();
        ImGui::PopStyleColor();
        ImGui::SameLine(0,style.ItemSpacing.x*1.5f);
        ImGui::SetCursorScreenPos({group.x+groupWidth+style.ItemSpacing.x*1.5f,group.y});
        ImGui::SetNextItemWidth(100);ImGui::SliderFloat("##fps",&playbackFps,1,60,"%.0f fps");
        Tip("Playback speed");ImGui::SameLine();
        if(RayFEM::UI::ToggleButton(liveLabel,follow)) {follow=!follow;playing=false;}
        Tip(follow?"Following incoming frames. Click to stay on the current frame.":"Jump to and follow the newest frame");
        if(Current()) {
            // The current frame, right-aligned on the same row when it fits.
            const auto label=TimelineLabel();
            const float labelWidth=ImGui::CalcTextSize(label.c_str()).x;
            const float rightEdge=ImGui::GetCursorPosX()+ImGui::GetContentRegionAvail().x;
            if(RayFEM::UI::NextInlineItem(labelWidth+style.ItemSpacing.x)) ImGui::SameLine(std::max(ImGui::GetCursorPosX(),rightEdge-labelWidth));
            ImGui::AlignTextToFramePadding();ImGui::TextDisabled("%s",label.c_str());
            char retained[48];
            std::snprintf(retained,sizeof(retained),"%d / %d",selected+1,static_cast<int>(frames.size()));
            const float countWidth=ImGui::CalcTextSize(retained).x;
            const float trackWidth=std::max(40.0f,ImGui::GetContentRegionAvail().x-countWidth-style.ItemSpacing.x*2);
            auto describe=[&](int index) {
                const auto& metadata=frames[std::clamp(index,0,static_cast<int>(frames.size())-1)]->metadata;
                char text[96];std::snprintf(text,sizeof(text),"Frame %u  ·  t %.6g",metadata.frame_index,metadata.sim_time);
                return std::string(text);
            };
            if(RayFEM::UI::Scrubber("##frame",&selected,static_cast<int>(frames.size()),trackWidth,describe)) {follow=false;playing=false;}
            ImGui::SameLine(0,style.ItemSpacing.x*2);
            ImGui::AlignTextToFramePadding();ImGui::TextDisabled("%s",retained);
            if(ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))ImGui::SetTooltip("Position among the retained frames");
        }
        ImGui::EndDisabled();
        ImGui::EndChild();
        ImGui::PopStyleVar(2);
        ImGui::PopStyleColor(2);
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
        viewportOrigin=origin;viewportSize=size;
        bool hovered=RayFEM::CameraViewportContains(mouse,{origin.x,origin.y,size.x,size.y}) &&
            IsWindowFocused() && !rayPlacement.Covers(mouse) &&
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
        // Rays are drawn on the last displayed surface.
        const ViewCache* pickEntry=cache.empty()?nullptr:&cache.back();
        auto pickSurface=[&](Ray ray)->std::optional<Vector3> {
            if(!pickEntry || !showSurface) return std::nullopt;
            const auto hit=pickEntry->picker.Pick(ray);
            return hit?std::optional<Vector3>{hit->position}:std::nullopt;
        };
        bool profileDrag=!pickingDof && profiles.HandleInput(origin,size,camera,meshHovered,pickSurface);
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
                // While placing a ray, a click chooses that endpoint's element.
                if(rayPlacement.WantsElement()) {if(hit)rayPlacement.ChooseElement(hit->element);}
                else {
                    elements.Select(hit?std::optional<int>{hit->element}:std::nullopt,elementPressAdditive);
                    if(hit)inspectorTab=2;
                }
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
        if(pickingDof && !ImGui::IsAnyItemActive() && ImGui::IsKeyPressed(ImGuiKey_Escape,false) &&
           !ImGui::IsPopupOpen(nullptr,ImGuiPopupFlags_AnyPopupId|ImGuiPopupFlags_AnyPopupLevel))pickingDof=false;
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
            if (showDofs) DrawDofMarkers(*active,{float(width),float(height)});
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
        surfaceTrace.DrawOverlay(origin,size,camera);
        if(rayPlacement.Active() && Current()) {
            const Matrix viewProjection=MatrixMultiply(MatrixLookAt(camera.position,camera.target,camera.up),
                MatrixPerspective(camera.fovy*DEG2RAD,double(width)/double(height),rlGetCullDistanceNear(),rlGetCullDistanceFar()));
            auto project=[&](Vector3 p,ImVec2& out) {
                const Quaternion q=QuaternionTransform({p.x,p.y,p.z,1},viewProjection);
                if(!(q.w>1e-6f))return false;
                out={origin.x+(q.x/q.w+1)*.5f*width,origin.y+(1-q.y/q.w)*.5f*height};
                return true;
            };
            try {rayPlacement.DrawMainMarkers(ImGui::GetWindowDrawList(),*Current()->mesh,RenderSettings(Current()),project);}
            catch(const std::exception& e){error=e.what();}
        }
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
                    if(active->rangeSkippedElements)
                        ImGui::Text("Large mesh: %zu elements not sampled; limits are approximate.",active->rangeSkippedElements);
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
        DrawViewportMessages(active,origin,size,orientation);
        RayFEM::UI::DrawOrientationCube(camera,orientation.cube,mouse,hovered && !existingDrag);
        RayFEM::UI::DrawCameraRotationArrows(orientation,mouse,hovered && !existingDrag,rotationIcons);
    }

    // Text overlays on the viewport: active display states, the picking hint,
    // errors, and empty-scene guidance. They stay visible on every tab.
    void DrawViewportMessages(const ViewCache* active,ImVec2 origin,ImVec2 size,
                              const RayFEM::UI::OrientationControlsLayout& orientation) {
        auto* draw=ImGui::GetWindowDrawList();
        const auto& palette=RayFEM::UI::Palette();
        ImFont* font=ImGui::GetFont();
        const float fontSize=ImGui::GetFontSize();
        const float left=origin.x+(active && coloring!=0?162.0f:12.0f);
        const float right=std::max(left+120,orientation.bounds.width>0?orientation.bounds.x-12:origin.x+size.x-12);
        float y=origin.y+12;
        auto count=[](std::size_t n,const char* singular,const char* plural) {
            return std::to_string(n)+" "+(n==1?singular:plural);
        };

        struct Chip { std::string label; const char* tip; };
        std::vector<Chip> chips;
        if(!server.receiving())
            chips.push_back({ICON_FA_PAUSE "  Stream paused","New frames are skipped. Resume from the status at the top right or the Info menu."});
        if(const auto frame=Current()) {
            if(clip) chips.push_back({ICON_FA_SCISSORS "  Clipped","Clipping is on. Change it under View > Clipping."});
            if(coloring==0 && !automaticColorRange)
                chips.push_back({ICON_FA_LOCK "  Fixed color range","Colors use fixed limits across frames. Change them under View > Color scale."});
            if(!excludedMaterials.empty())
                chips.push_back({ICON_FA_LAYER_GROUP "  "+count(excludedMaterials.size(),"material hidden","materials hidden"),
                                 "Hidden materials are excluded from rendering and analysis. Change them under View > Regions > Materials."});
            if(!elements.Hidden().empty())
                chips.push_back({ICON_FA_EYE_SLASH "  "+count(elements.Hidden().size(),"element hidden","elements hidden"),
                                 "Hidden elements are excluded from rendering and analysis. Show them again from the Elements tab."});
            if(elementExplosion>0)
                chips.push_back({ICON_FA_UP_RIGHT_AND_DOWN_LEFT_FROM_CENTER "  Exploded view",
                                 "Elements are shrunk around their centers. Set Explode elements to zero under View > Appearance to restore the mesh."});
            if(frame->displacement && !deform)
                chips.push_back({"Displacement off","The reference geometry is shown. Turn on Apply displacement under View > Deformation."});
            else if(frame->displacement && displacementScale!=1) {
                char label[48];std::snprintf(label,sizeof(label),"Displacement ×%.2f",displacementScale);
                chips.push_back({label,"Displacement is scaled. Change it under View > Deformation."});
            }
        }
        const float chipHeight=fontSize+8,chipPadding=9;
        float x=left;
        for(const auto& chip:chips) {
            const float width=ImGui::CalcTextSize(chip.label.c_str()).x+2*chipPadding;
            if(x>left && x+width>right) {x=left;y+=chipHeight+6;}
            const ImVec2 a{x,y},b{x+width,y+chipHeight};
            draw->AddRectFilled(a,b,RayFEM::UI::WithAlpha(palette.plot_background,235),chipHeight*.5f);
            draw->AddRect(a,b,palette.border,chipHeight*.5f);
            draw->AddText({x+chipPadding,y+4},palette.text,chip.label.c_str());
            if(ImGui::IsMouseHoveringRect(a,b) && !ImGui::IsPopupOpen(nullptr,ImGuiPopupFlags_AnyPopupId|ImGuiPopupFlags_AnyPopupLevel))
                ImGui::SetTooltip("%s",chip.tip);
            x+=width+6;
        }
        if(!chips.empty()) y+=chipHeight+8;

        // Rebuilds over a shown surface get a small corner spinner, only once
        // they last long enough to notice, so streaming does not flicker it.
        if(active && surfaceBuilding) {if(buildingSince<0)buildingSince=GetTime();}
        else buildingSince=-1;
        if(buildingSince>=0 && GetTime()-buildingSince>.25) {
            const ImVec2 center{origin.x+size.x-18,origin.y+size.y-18};
            RayFEM::UI::DrawSpinner(draw,center,6,palette.muted,GetTime());
            if(ImGui::IsMouseHoveringRect({center.x-10,center.y-10},{center.x+10,center.y+10}) &&
               !ImGui::IsPopupOpen(nullptr,ImGuiPopupFlags_AnyPopupId|ImGuiPopupFlags_AnyPopupLevel))
                ImGui::SetTooltip("Updating. The last finished surface is shown while the current frame and settings are built.");
        }

        // A tinted, wrapped message box spanning the free width at the top.
        auto banner=[&](const char* icon,const char* text,ImU32 tone) {
            const float padding=10,iconWidth=icon?ImGui::CalcTextSize(icon).x+8:0;
            const float wrap=std::max(40.0f,right-left-2*padding-iconWidth);
            const ImVec2 textSize=font->CalcTextSizeA(fontSize,FLT_MAX,wrap,text);
            const ImVec2 a{left,y},b{std::min(right,left+textSize.x+iconWidth+2*padding),y+textSize.y+2*7};
            draw->AddRectFilled(a,b,RayFEM::UI::WithAlpha(palette.plot_background,240),RayFEM::UI::OverlayRounding);
            draw->AddRectFilled(a,b,RayFEM::UI::WithAlpha(tone,40),RayFEM::UI::OverlayRounding);
            draw->AddRect(a,b,RayFEM::UI::WithAlpha(tone,160),RayFEM::UI::OverlayRounding);
            if(icon)draw->AddText({left+padding,y+7},tone,icon);
            draw->AddText(font,fontSize,{left+padding+iconWidth,y+7},palette.text,text,nullptr,wrap);
            y=b.y+8;
        };
        if(rayPlacement.Current()==RayFEM::UI::RayPlacement::Step::ChooseElement) {
            using Kind=RayFEM::UI::RayPlacement::Kind;
            std::string text;
            if(rayPlacement.Placing()==Kind::Center) text="Click the element whose center becomes the path center. Esc to cancel.";
            else if(rayPlacement.Placing()==Kind::Point) text="Click the element where the plane center goes. Esc to cancel.";
            else if(rayPlacement.Placing()==Kind::Path)
                text="Click an element on the surface for path point "+std::to_string(rayPlacement.Endpoint()+1)+
                     (rayPlacement.Single()?". Esc to cancel.":". Finish below when done; Esc to cancel.");
            else text=rayPlacement.Endpoint()==0?"Click the element where the ray starts (step 1 of 2). Drag to orbit; Esc to cancel.":
                                                 "Click the element where the ray ends (step 2 of 2). Drag to orbit; Esc to cancel.";
            banner(ICON_FA_CROSSHAIRS,text.c_str(),palette.accent);
            if(!rayPlacement.Message().empty()) banner(ICON_FA_TRIANGLE_EXCLAMATION,rayPlacement.Message().c_str(),palette.warning);
        }
        else if(rayPlacement.Current()==RayFEM::UI::RayPlacement::Step::PlacePoint)
            banner(ICON_FA_CROSSHAIRS,"Place the point on the element in the inset viewer, or click another element here.",palette.accent);
        if(pickingDof)
            banner(ICON_FA_CROSSHAIRS,"Click a DOF marker to track its coefficient.  Press Esc or Cancel pick to stop.",palette.accent);
        if(!error.empty()) banner(ICON_FA_TRIANGLE_EXCLAMATION,error.c_str(),palette.warning);
        if(Current() && !active && !surfaceBuilding)
            banner(error.empty()?ICON_FA_TRIANGLE_EXCLAMATION:nullptr,"Choose a supported field operation in the View tab.",palette.warning);

        // Centered guidance for an empty viewport.
        auto centered=[&](std::initializer_list<std::pair<std::string,ImU32>> lines) {
            const float lineHeight=fontSize+6;
            float lineY=origin.y+size.y*.5f-lineHeight*lines.size()*.5f;
            float widest=0;
            for(const auto& line:lines) widest=std::max(widest,ImGui::CalcTextSize(line.first.c_str()).x);
            const ImVec2 center{origin.x+size.x*.5f,origin.y+size.y*.5f};
            const ImVec2 half{widest*.5f+18,lineHeight*lines.size()*.5f+12};
            draw->AddRectFilled({center.x-half.x,center.y-half.y},{center.x+half.x,center.y+half.y},
                                RayFEM::UI::WithAlpha(palette.plot_background,235),RayFEM::UI::OverlayRounding);
            for(const auto& [text,color]:lines) {
                const float width=ImGui::CalcTextSize(text.c_str()).x;
                draw->AddText({origin.x+(size.x-width)*.5f,lineY},color,text.c_str());
                lineY+=lineHeight;
            }
        };
        if(!Current()) {
            const std::string stream="Stream frames to "+opts.listen+":"+std::to_string(opts.port);
            if(RayFEM::SerifLoadingAvailable())
                centered({{ICON_FA_FOLDER_OPEN "  No data yet",palette.text},
                          {"Open a .serif archive (Ctrl/Cmd+O) or drop one here,",palette.muted},
                          {"or s"+stream.substr(1)+".",palette.muted}});
            else
                centered({{ICON_FA_TOWER_BROADCAST "  Waiting for frames",palette.text},{stream+".",palette.muted}});
        } else if(!active && surfaceBuilding)
            centered({{ICON_FA_ARROWS_ROTATE "  Building surface",palette.text},{"Evaluating the field on the mesh.",palette.muted}});
        else if(active && !active->vertices && !surfaceBuilding)
            centered({{ICON_FA_EYE_SLASH "  No visible elements",palette.text},
                      {"Check hidden materials, hidden elements, or the clipping plane.",palette.muted}});
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

    static std::size_t DofStride(const ViewCache& active) {return std::max<size_t>(1,(active.dofs.size()+19999)/20000);}

    void EnsureDofs(ViewCache& active) {
        try {
            if (!active.dofs_ready) {
                if (auto* gf=Field(active.frame)) active.dofs=RayFEM::SampleDofs(*active.frame->mesh,*gf,RenderSettings(active.frame),20000,active.retainedElements.empty()?nullptr:&active.retainedElements);
                active.dofs_ready=true;
                TrimViewCache();
            }
        } catch (const std::exception& e) {error=e.what();active.dofs_ready=true;}
    }

    // GPU markers inside the 3D pass; rebuilt only for new DOFs or tracks.
    void DrawDofMarkers(ViewCache& active,Vector2 viewport) {
        EnsureDofs(active);
        const auto colors=tracking.DofColors(*active.frame,fieldName);
        std::vector<std::pair<int,ImU32>> sorted(colors.begin(),colors.end());
        std::sort(sorted.begin(),sorted.end());
        std::string tracks;
        for(const auto& [dof,color]:sorted) tracks+=std::to_string(dof)+':'+std::to_string(color)+';';
        if(!active.dofsUploaded || active.dofTracks!=tracks) {
            UnloadModel(active.dofModel);active.dofModel={};
            try {active.dofModel=dofRenderer.Upload(active.dofs,DofStride(active),colors);}
            catch(const std::exception& e) {error=e.what();}
            active.dofsUploaded=true;active.dofTracks=tracks;
        }
        const auto toColor=[](ImU32 c){return Color{static_cast<unsigned char>(c&0xff),static_cast<unsigned char>((c>>8)&0xff),
                                                     static_cast<unsigned char>((c>>16)&0xff),static_cast<unsigned char>((c>>24)&0xff)};};
        const RayFEM::ClipPlane plane{slicePoint,sliceNormal};
        dofRenderer.Draw(active.dofModel,viewport,toColor(RayFEM::UI::Palette().dof),toColor(RayFEM::UI::Palette().dof_ring),
                         clip && clipMode==0?&plane:nullptr);
    }

    // Hover details and picking. Markers themselves are drawn on the GPU.
    void DrawDofs(ViewCache& active,ImVec2 origin,ImVec2 size,bool hovered) {
        EnsureDofs(active);
        if (!hovered) return;
        const RayFEM::DofPoint* nearest=nullptr;
        float distance=81;
        auto mouse=ImGui::GetMousePos();
        const size_t stride=DofStride(active);
        // One view-projection for every marker, as GetWorldToScreenEx builds per point.
        const int width=target.texture.width,height=target.texture.height;
        const Matrix viewProjection=MatrixMultiply(MatrixLookAt(camera.position,camera.target,camera.up),
            MatrixPerspective(camera.fovy*DEG2RAD,double(width)/double(height),rlGetCullDistanceNear(),rlGetCullDistanceFar()));
        const Vector3 forward=Vector3Subtract(camera.target,camera.position);
        for (size_t i=0;i<active.dofs.size();i+=stride) {
            const auto& point=active.dofs[i];
            if (Vector3DotProduct(Vector3Subtract(point.position,camera.position),forward)<=0) continue;
            if (clip && clipMode==0 && Vector3DotProduct(Vector3Subtract(point.position,slicePoint),sliceNormal)>0) continue;
            const Quaternion clipSpace=QuaternionTransform({point.position.x,point.position.y,point.position.z,1},viewProjection);
            const ImVec2 screen={origin.x+(clipSpace.x/clipSpace.w+1)*.5f*width,origin.y+(1-clipSpace.y/clipSpace.w)*.5f*height};
            float d=(screen.x-mouse.x)*(screen.x-mouse.x)+(screen.y-mouse.y)*(screen.y-mouse.y);
            if (d<distance) {distance=d;nearest=&point;}
        }
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
    RayFEM::UI::FramePipeline pipeline{server};
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
    int inspectorTab=-1,inspectorPage=0,plotTab=-1;
    bool showPlots=true,pickingDof=false;
    float plotPanelHeight=0;
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
    std::string fieldName,error,failedKey,requestedKey;
    bool surfaceBuilding=false;
    double buildingSince=-1;
    bool profileClipping=false,savedClip=false;
    Vector3 savedSlicePoint{},savedSliceNormal{1,0,0};
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
    RayFEM::UI::DofRenderer dofRenderer;
    RayFEM::UI::RayPlacement rayPlacement;
    ImVec2 viewportOrigin{},viewportSize{};
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
