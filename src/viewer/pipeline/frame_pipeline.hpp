#pragma once

#include "interaction/element_picker.hpp"
#include "pipeline/indexed_surface.hpp"
#include "libRayfem/frame/render.hpp"
#include "libRayfem/mesh/clipping.hpp"
#include "libRayfem/mesh/color_scale.hpp"
#include "libRayfem/mesh/contours.hpp"
#include "libRayfem/mesh/converter.hpp"
#include "libRayfem/mesh/field_range.hpp"
#include "libRayfem/network/server.hpp"
#include "libRayfem/threading.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// Threading model. MFEM meshes, spaces and fields are not safe to use from two
// threads at once, so every frame exists twice: the viewer's copy, used only
// by the UI thread, and RenderFrame::compute, used only by the worker below.
// The worker decodes the stream and builds surfaces; the UI thread draws the
// last finished surface while a newer one is built. Collections and spaces
// are constructed under FiniteElementConstructionMutex (see threading.hpp).
namespace RayFEM::UI {

// A space shared by copies of streamed fields that use the same layout.
struct SharedSpace {
    std::shared_ptr<mfem::Mesh> mesh;
    std::unique_ptr<mfem::FiniteElementCollection> collection;
    std::unique_ptr<mfem::FiniteElementSpace> space;
};

namespace detail {
struct SpaceHolder { std::shared_ptr<SharedSpace> shared; };
}
// Holds its space alive; the base order destroys the field before the space.
class SharedSpaceGridFunction : private detail::SpaceHolder, public mfem::GridFunction {
public:
    explicit SharedSpaceGridFunction(std::shared_ptr<SharedSpace> space)
        : detail::SpaceHolder{std::move(space)}, mfem::GridFunction(shared->space.get()) {}
};

// Spaces copied onto one destination mesh, reused across frames. Streamed
// frames otherwise rebuild an identical space for every field of every frame.
class SpaceCache {
public:
    std::shared_ptr<SharedSpace> Get(const mfem::FiniteElementSpace& source, const std::shared_ptr<mfem::Mesh>& mesh) {
        if (!mesh) throw std::invalid_argument("Space copy needs a destination mesh");
        // NURBS spaces carry their own knot data; copy those every time.
        const bool cacheable = !source.GetNURBSext();
        const std::string key = cacheable ? Key(source, *mesh) : std::string{};
        if (cacheable) {
            if (auto found = entries_.find(key); found != entries_.end())
                if (auto space = found->second.lock(); space && space->mesh == mesh) return space;
        }
        auto space = std::make_shared<SharedSpace>();
        space->mesh = mesh;
        {
            std::lock_guard construction(RayFEM::FiniteElementConstructionMutex());
            space->collection.reset(mfem::FiniteElementCollection::New(source.FEColl()->Name()));
            space->space = std::make_unique<mfem::FiniteElementSpace>(source, mesh.get(), space->collection.get());
        }
        // A null mesh would make MFEM reuse the source mesh, sharing it
        // between threads; require the copy to live on the destination.
        if (space->space->GetMesh() != mesh.get() || space->space->GetVSize() != source.GetVSize())
            throw std::runtime_error("Copied finite element space does not match its source");
        if (cacheable) {
            std::erase_if(entries_, [](const auto& entry) { return entry.second.expired(); });
            entries_[key] = space;
        }
        return space;
    }
private:
    static std::string Key(const mfem::FiniteElementSpace& source, const mfem::Mesh& mesh) {
        return std::to_string(reinterpret_cast<std::uintptr_t>(&mesh)) + '|' + source.FEColl()->Name() + '|' +
               std::to_string(source.GetVDim()) + '|' + std::to_string(source.GetOrdering()) + '|' +
               std::to_string(source.GetVSize());
    }
    std::unordered_map<std::string, std::weak_ptr<SharedSpace>> entries_;
};

// Copy a frame onto a mesh that is a deep copy of the frame's own mesh.
inline std::shared_ptr<RenderFrame> CopyFrame(const RenderFrame& source, std::shared_ptr<mfem::Mesh> mesh,
                                              SpaceCache& spaces) {
    if (!mesh) throw std::invalid_argument("Frame copy needs a destination mesh");
    auto frame = std::make_shared<RenderFrame>();
    frame->metadata = source.metadata;
    frame->schema = source.schema;
    frame->mesh_id = source.mesh_id;
    frame->session_id = source.session_id;
    frame->mesh = std::move(mesh);
    frame->mesh_revision = source.mesh_revision;
    auto copy_field = [&](const mfem::GridFunction& field) -> std::unique_ptr<mfem::GridFunction> {
        auto result = std::make_unique<SharedSpaceGridFunction>(spaces.Get(*field.FESpace(), frame->mesh));
        static_cast<mfem::Vector&>(*result) = field;
        return result;
    };
    for (const auto& field : source.fields)
        frame->fields.push_back({field.name, copy_field(*field.gf), {}, field.supported_materials});
    if (source.displacement) frame->displacement = copy_field(*source.displacement);
    std::size_t data = 0;
    for (const auto& field : source.fields) data += field.gf->Size()*sizeof(mfem::real_t);
    if (source.displacement) data += source.displacement->Size()*sizeof(mfem::real_t);
    // Both copies are retained; count both against the history budget.
    frame->estimated_bytes = source.estimated_bytes + data;
    frame->estimated_mesh_bytes = 2*source.estimated_mesh_bytes;
    return frame;
}

inline std::shared_ptr<mfem::Mesh> CopyMesh(const mfem::Mesh& mesh) {
    std::lock_guard construction(RayFEM::FiniteElementConstructionMutex());
    return std::make_shared<mfem::Mesh>(mesh, true);
}

// One surface the viewer wants: a frame plus every setting that shapes it.
struct SurfaceRequest {
    std::shared_ptr<RenderFrame> frame;  // viewer frame; its compute twin is evaluated
    std::string key;                     // identifies the result for the view cache
    std::string appearance;              // color settings when requested
    std::string field, clipping;
    int view = 0, component = 0, refinement = 3;
    float displacement = 0;
    RenderOptions options;               // displacement is resolved on the twin
    bool deform = false;
    bool clip = false;
    ClipPlane plane{};
    ClipMode clip_mode = ClipMode::CutThrough;
    CutAlgorithm cut_algorithm = CutAlgorithm::Refined;
    bool automatic_range = true;
    double minimum = 0, maximum = 1;
    bool boundaries = false;
    bool dofs = false;
    int contours = 0;  // contour lines to extract with the field colors; 0 for none
};

struct SurfaceResult {
    SurfaceRequest request;
    // Bounds, limits and counts only; the vertex data moves into indexed.
    SurfaceData data;
    IndexedSurface indexed;
    std::size_t vertex_count = 0;
    std::vector<LineSegment> boundaries;
    bool boundaries_ready = false;
    std::vector<bool> retained;
    std::vector<DofPoint> dofs;
    bool dofs_ready = false;
    ElementPicker picker;
    bool has_range = false;
    ColorScaleMapping mapping{};
    // Field value per triangle-soup vertex, kept so contours can be redrawn
    // for new levels without rebuilding the surface.
    std::vector<float> soup_values;
    std::vector<LineSegment> contours;
    std::vector<double> contour_levels;
    double volume_min = 0, volume_max = 0;
    std::size_t range_samples = 0, skipped_elements = 0;
    bool budget_limited = false;
    std::vector<double> distribution;
    double milliseconds = 0;
    std::string error;
};

class FramePipeline {
public:
    // fetch is called on the worker thread for newly decoded frames.
    using Fetch = std::function<std::vector<std::shared_ptr<RenderFrame>>(size_t)>;
    explicit FramePipeline(Fetch fetch) : fetch_(std::move(fetch)),
        worker_([this](std::stop_token stop) { Loop(stop); }) {}
    explicit FramePipeline(Server& server)
        : FramePipeline([&server](size_t count) { return server.fetchFrames(count); }) {}
    ~FramePipeline() { Stop(); }
    FramePipeline(const FramePipeline&) = delete;
    FramePipeline& operator=(const FramePipeline&) = delete;

    void Stop() {
        if (!worker_.joinable()) return;
        worker_.request_stop();
        wake_.notify_all();
        worker_.join();
    }

    // UI thread: viewer copies of newly decoded frames, oldest first.
    std::vector<std::shared_ptr<RenderFrame>> TakeFrames() {
        std::deque<Arrival> arrivals;
        { std::lock_guard lock(mutex_); arrivals.swap(arrivals_); }
        std::vector<std::shared_ptr<RenderFrame>> frames;
        for (auto& arrival : arrivals) {
            auto frame = CopyFrame(*arrival.compute, arrival.viewer_mesh, spaces_);
            frame->compute = std::move(arrival.compute);
            frames.push_back(std::move(frame));
        }
        return frames;
    }

    // UI thread: replaces any request the worker has not started.
    void Request(SurfaceRequest request) {
        { std::lock_guard lock(mutex_); pending_ = std::move(request); }
        wake_.notify_all();
    }

    std::optional<SurfaceResult> TakeResult() {
        std::lock_guard lock(mutex_);
        if (results_.empty()) return std::nullopt;
        auto result = std::move(results_.front());
        results_.pop_front();
        return result;
    }

    bool Busy() const { std::lock_guard lock(mutex_); return building_ || pending_.has_value(); }

    // UI thread: run panel work on the worker, replacing any job still queued
    // on the same channel. Jobs must use only compute twins and their own
    // captured state, and report back through state they share with the UI.
    void Submit(const std::string& channel, std::function<void()> job) {
        { std::lock_guard lock(mutex_); jobs_[channel] = std::move(job); }
        wake_.notify_all();
    }

    // UI thread: run work that must not overlap the worker's MFEM use (for
    // example loading an archive) on the worker, and wait for it.
    template<class F> auto RunOnWorker(F&& work) -> decltype(work()) {
        std::packaged_task<decltype(work())()> task(std::forward<F>(work));
        auto result = task.get_future();
        { std::lock_guard lock(mutex_); tasks_.emplace_back([&task] { task(); }); }
        wake_.notify_all();
        return result.get();
    }

    // Worker thread (inside RunOnWorker): give viewer frames compute twins.
    void AttachComputeTwins(const std::vector<std::shared_ptr<RenderFrame>>& frames) {
        SpaceCache spaces;
        std::unordered_map<const mfem::Mesh*, std::shared_ptr<mfem::Mesh>> meshes;
        for (const auto& frame : frames) {
            auto& mesh = meshes[frame->mesh.get()];
            if (!mesh) mesh = CopyMesh(*frame->mesh);
            frame->compute = CopyFrame(*frame, mesh, spaces);
        }
    }

private:
    struct Arrival { std::shared_ptr<RenderFrame> compute; std::shared_ptr<mfem::Mesh> viewer_mesh; };
    struct MeshPair { std::weak_ptr<mfem::Mesh> compute, viewer; };
    struct RangeEntry {
        std::weak_ptr<RenderFrame> frame;
        std::string field;
        FieldView view;
        int component;
        std::vector<int> excluded, excluded_elements;
        FieldRange range;
    };

    void Loop(std::stop_token stop) {
        while (!stop.stop_requested()) {
            std::function<void()> task;
            std::optional<SurfaceRequest> request;
            {
                std::unique_lock lock(mutex_);
                wake_.wait_for(lock, std::chrono::milliseconds(2), [&] {
                    return stop.stop_requested() || !tasks_.empty() || pending_.has_value() || !jobs_.empty();
                });
                if (stop.stop_requested()) break;
                if (!tasks_.empty()) { task = std::move(tasks_.front()); tasks_.pop_front(); }
                else if (pending_) { request = std::move(pending_); pending_.reset(); building_ = true; }
                else if (!jobs_.empty()) { task = std::move(jobs_.begin()->second); jobs_.erase(jobs_.begin()); }
            }
            if (task) { task(); continue; }
            // New frames first: decoding is never skipped, but only the
            // newest surface request is ever built.
            Ingest();
            if (request) {
                auto result = Build(std::move(*request));
                std::lock_guard lock(mutex_);
                results_.push_back(std::move(result));
                building_ = false;
            }
        }
    }

    void Ingest() {
        for (auto& frame : fetch_(64)) {
            auto& pair = meshes_[frame->mesh.get()];
            auto viewer = pair.compute.lock() == frame->mesh ? pair.viewer.lock() : nullptr;
            if (!viewer) {
                std::erase_if(meshes_, [](const auto& entry) { return entry.second.compute.expired(); });
                viewer = CopyMesh(*frame->mesh);
                meshes_[frame->mesh.get()] = {frame->mesh, viewer};
            }
            std::lock_guard lock(mutex_);
            arrivals_.push_back({std::move(frame), std::move(viewer)});
        }
    }

    FieldRange VolumeRange(const std::shared_ptr<RenderFrame>& compute, mfem::GridFunction& field,
                           const SurfaceRequest& request, const RenderOptions& options) {
        std::erase_if(ranges_, [](const auto& entry) { return entry.frame.expired(); });
        auto found = std::find_if(ranges_.begin(), ranges_.end(), [&](const auto& entry) {
            return entry.frame.lock() == compute && entry.field == request.field && entry.view == options.view &&
                entry.component == options.component && entry.excluded == options.excluded_materials &&
                entry.excluded_elements == options.excluded_elements;
        });
        if (found != ranges_.end()) {
            std::rotate(found, found + 1, ranges_.end());
            return ranges_.back().range;
        }
        FieldRangeOptions sampling;
        sampling.distribution_samples = 1000;
        sampling.reduce_to_fit = true;
        auto range = EvaluateFieldRange(*compute->mesh, field, options, sampling);
        ranges_.push_back({compute, request.field, options.view, options.component,
                           options.excluded_materials, options.excluded_elements, range});
        if (ranges_.size() > 64) ranges_.pop_front();
        return range;
    }

    SurfaceResult Build(SurfaceRequest request) {
        SurfaceResult result;
        const auto start = std::chrono::steady_clock::now();
        try {
            auto compute = request.frame->compute;
            if (!compute || !compute->mesh) throw std::runtime_error("Frame has no compute copy");
            auto& mesh = *compute->mesh;
            mfem::GridFunction* field = nullptr;
            for (const auto& f : compute->fields) if (f.name == request.field) { field = f.gf.get(); break; }
            auto options = request.options;
            options.displacement = request.deform && compute->displacement ? compute->displacement.get() : nullptr;
            if (request.clip) {
                auto cut = BuildClippedSurface(mesh, field, options, request.plane, request.clip_mode, request.cut_algorithm);
                result.data = std::move(cut.surface);
                result.boundaries = std::move(cut.element_boundaries);
                result.boundaries_ready = true;
                result.retained = std::move(cut.retained_elements);
            } else {
                result.data = BuildSurface(mesh, field, options);
                if (request.boundaries) {
                    result.boundaries = BuildElementBoundaries(mesh, options);
                    result.boundaries_ready = true;
                }
            }
            // Keep the legend and its scale even when clipping hides the whole
            // field. An explicitly empty material selection has none.
            bool included = false;
            if (options.coloring == SurfaceColoring::Field && field) {
                const ElementSelection selection(mesh, options);
                for (int e = 0; e < mesh.GetNE() && !included; ++e) included = selection.Includes(e);
            }
            if (included) {
                const auto range = VolumeRange(compute, *field, request, options);
                result.mapping = request.automatic_range ?
                    ResolveColorScale(range.min_value, range.max_value, options.color_scale, range.samples) :
                    ResolveColorScale(request.minimum, request.maximum, options.color_scale);
                ApplyFieldColorRange(result.data, result.mapping, options.color_map, options.reverse_colors);
                result.volume_min = range.min_value;
                result.volume_max = range.max_value;
                result.range_samples = range.sample_count;
                result.budget_limited = range.budget_limited;
                result.skipped_elements = range.skipped_elements;
                result.distribution = range.samples;
                result.has_range = true;
                result.soup_values.assign(result.data.values.begin(), result.data.values.end());
                if (request.contours > 0) {
                    result.contour_levels = ContourLevels(result.mapping, request.contours);
                    result.contours = ExtractContours(result.data.positions, result.soup_values, result.contour_levels);
                }
            }
            if (request.dofs && field) {
                result.dofs = SampleDofs(mesh, *field, options, 20000, result.retained.empty() ? nullptr : &result.retained);
                result.dofs_ready = true;
            }
            result.vertex_count = result.data.positions.size()/3;
            result.indexed = WeldSurface(result.data);
            // The picker keeps the triangle positions; the rest is freed below.
            if (!result.data.positions.empty())
                result.picker.Build(std::move(result.data.positions), result.data.element_ids);
            auto& data = result.data;
            for (auto* buffer : {&data.positions, &data.normals}) std::vector<float>().swap(*buffer);
            std::vector<unsigned char>().swap(data.colors);
            std::vector<double>().swap(data.values);
            for (auto* buffer : {&data.material_ids, &data.boundary_ids, &data.element_ids}) std::vector<int>().swap(*buffer);
        } catch (const std::exception& e) { result.error = e.what(); }
        result.milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        result.request = std::move(request);
        return result;
    }

    Fetch fetch_;
    mutable std::mutex mutex_;
    std::condition_variable wake_;
    std::deque<Arrival> arrivals_;
    std::optional<SurfaceRequest> pending_;
    bool building_ = false;
    std::deque<SurfaceResult> results_;
    std::deque<std::function<void()>> tasks_;
    std::map<std::string, std::function<void()>> jobs_;
    std::unordered_map<const mfem::Mesh*, MeshPair> meshes_;  // worker only
    std::deque<RangeEntry> ranges_;                          // worker only
    SpaceCache spaces_;                                      // UI thread only
    std::jthread worker_;  // last: starts after the members above exist
};

} // namespace RayFEM::UI
