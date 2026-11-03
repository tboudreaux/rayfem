# Using the viewer

## Appearance and navigation

The **Light mode / Dark mode** button changes the controls, viewport, plots, legends, DOF markers, and flow overlays together. Scientific surface and heatmap colors retain their data meaning. `RayFEM --light` starts in light mode.

The orientation cube in the viewport's upper-right corner follows the camera. Click a face, edge, or corner for one of 26 views; the camera eases smoothly into place while preserving its target and zoom. Axis signs label the faces (Y is up, +Z is front). Drag the cube to turn the view: it follows the pointer like a dragged mesh, faster, so a drag across the cube turns about 90°. A press that does not move is a click. Dragging, panning, zooming, or fitting the mesh interrupts the transition.

**Spin** in the toolbar (or `S` over the viewport) keeps the view turning, as in GLVis; only the camera moves, never the data or its time. The menu beside it chooses the axis — screen vertical (a turntable), screen horizontal, world X, Y or Z, or one of the model's axes — and the rate in degrees per second; negative rates turn the other way. A model axis turns the view about the model's own axis line. With **Throw to spin** on, releasing a drag (in the view or on the cube) while the pointer still moves keeps the view turning that way at that speed. A click in the view, Fit, Home or a cube click stops the spin.

When a client reports axes for its model (see [model axes](streaming.md#model-axes)), **View → Appearance → Model axes** draws each one as a rod through the model with an arrowhead and its name at the positive end; the model hides the parts inside it. **Orient by … axis** (on by default) makes Fit and Home put the first axis up the screen, looking from 20° above the model's equator, instead of down at the reference plane.

**View → Lighting** sets how the surface is shaded. **Flat color** draws exact colormap colors with no shading, so colors read as values; **Matte** lights smoothly varying normals so curved surfaces look round; **Faceted** gives each drawn triangle one normal, showing the tessellation; **Glossy** adds a specular highlight with a set strength and sharpness. The light is one directional source: drag it on the pad (the center shines from the camera, the rim from the side, and a hollow marker is behind the mesh) or use the presets. **With view** keeps it in place on screen as you orbit; **In scene** fixes it to the mesh. **Ambient** is the light every surface gets evenly. Both sides of a surface are lit alike. Figures and movies use the same lighting.

**View → Appearance → Axes** draws the axes that exported figures use over the live view: Cartesian rulers along the mesh's bounding box, or cylindrical, spherical or polar rings about a chosen origin and axis, with ticks on round numbers. Behind the mesh they are dashed or hidden. A figure exported while they show starts with the same style, and movies draw them too (see below).

The arrows below the cube rotate the image counterclockwise/clockwise around the viewing direction: single arrows move 5° and double arrows move 45°, with the same smooth easing. Repeated clicks accumulate their full increments during animation. Their artwork comes from `assets/rotation`. Rebuilds automatically regenerate the embedded icons when an SVG changes if `rsvg-convert` is available. Builds without it use the checked-in artwork after verifying that it matches the SVGs; stale artwork causes an explicit build error. Run `python3 tools/generate_rotation_icons.py` to refresh that portable fallback after editing assets. Installed viewers do not need an SVG renderer.

**View → Appearance → Keep reference plane** keeps the grid visible alongside loaded meshes. It is off by default, so the grid normally disappears when a mesh arrives; the setting can also be changed while waiting for frames. The plane defaults to XY at Z = 0. **Plane orientation** offers XY/XZ/YZ presets and X/Y/Z rotation angles about the world origin. The **Home** button between the rotation arrows (or the Home key over the viewport) smoothly returns to a view looking down at the plane from 30° above it, keeping the current focus and zoom. The initial view and **Fit** use the same home orientation.

Surface detail is automatic by default and follows the mesh and displacement orders. Curved surfaces use normals evaluated from their geometry, so high-order Stroid and NURBS meshes retain their shape and curved appearance. Under **Appearance → Detail**, disable **Automatic detail** to choose the number of subdivisions manually. This changes only the displayed triangulation; it does not refine or replace the MFEM mesh.

The color scale defaults to the current field's range throughout the selected volume, including the interior and regions hidden by clipping. It respects hidden elements, material exclusions, and an imported field's saved material support. Ranges are sampled from the finite element field, including interior points and element nodes; they are numerical estimates rather than guaranteed continuous extrema. Rotation, clipping, deformation display, and surface-detail changes reuse the same range. Under **Color scale**, turn off **Automatic volume range** and enter minimum/maximum values to keep a fixed scale across frames. Changing the field or evaluated component resets automatic scaling. Hover over the legend for precise limits and the volume sample count.

**View → Color scale → Color map** selects Viridis (default), Plasma, Inferno, Cividis, Coolwarm, or legacy Jet, with optional reversal. Surface colors, clipping sections, legends, 2D profile heatmaps, and flow magnitude colors share the selected map. The sequential maps use Matplotlib's published lookup tables; Coolwarm is a diverging map (set symmetric manual limits to put zero at its midpoint). Material/boundary ID colors and categorical tracking lines retain their separate palettes. Profile heatmaps use their sampled profile range, and flow colors use the magnitude range shown in the flow controls. The library exposes `ColorMap`, `SampleColorMap`, and `FieldColor` in `libRayfem/mesh/colormap.hpp`; set `RenderOptions::color_map` and `reverse_colors` for CPU surface generation.

The **Scale** selector offers **Linear** (default), **Signed log**, and **ZScale**. Signed log uses `sign(x) * log1p(abs(x)/threshold)`, which is continuous at zero and supports either sign. Its automatic threshold is 1% of the largest absolute display limit; a custom threshold is in field units. Legend labels and hover values invert the selected transform. ZScale uses the [IRAF/DS9 ranked-sample line fit](https://iraf.readthedocs.io/en/latest/tasks/images/tv/display.html#zscale-algorithm), with iterative outlier rejection and adjustable contrast (default 0.25), then maps the fitted limits linearly. Higher contrast narrows the limits; zero uses the full range. Turning off **Automatic volume range** freezes the displayed limits, including fitted ZScale limits, across frames. The legend tooltip retains the full volume extrema.

ZScale takes up to 1,000 deterministic samples spread throughout the selected field volume, including clipped regions. This samples reference-cell lattices and FE nodes; it approximates the distribution and is not weighted by physical volume. Material exclusions still apply. Flow uses its whole-volume magnitude distribution; 2D heatmaps use their valid plane samples. Scalar profile and tracking axes remain unchanged. Library consumers can set `RenderOptions::color_scale`, or use `ResolveColorScale`, `ColorScaleFraction`, and `ColorScaleValue` from `libRayfem/mesh/color_scale.hpp`. To reproduce the viewer's volume-based limits, request `FieldRangeOptions::distribution_samples`, resolve the mapping from the resulting `FieldRange::samples`, then call `ApplyFieldColorRange(surface, mapping)`.

## Toolbar

The toolbar keeps to one row: **Fit** and **Spin** (with its menu), the analysis tools (Pick DOF, 1D and 2D profiles, Surface trace, Flow, Annotate), the Tracks and Plots panels, and **Export** at the right, whose menu holds **Figure…** (Ctrl/Cmd+E) and **Movie…**. When the labels do not fit, every button shows only its icon; hover for its name.

The Figure and Movie windows open pinned in a column right of the view, which stays usable while they are open: the figure preview follows the view as you orbit, pan or zoom. Drag the column's edge to resize it (double-click to restore its width); with both open they share it as tabs. The pin button in the column's header floats a window over the viewer instead, and the pin button in a floating window's title bar pins it again.

## Annotations

**Annotate** opens a window for lines and labels on the model. **Line** then two clicks on the surface draws a line between the points; **Label** then one click places text there. Each annotation has a visibility checkbox, a color, editable text (optional for a line, shown at its middle) and exact coordinates; Esc cancels placing. Annotations are drawn in the view and in movies, in world coordinates, so they stay put across frames; their text is drawn in movies too.

Text can show values of the frame on screen: `{name}` fills in one of the schema's values as the client sent it, and `{name:.3f}`, `{name:.2e}` or `{name:d}` format a number printf-style. `{frame}`, `{time}`, `{label}`, `{tags}` and `{caption}` (the timeline's caption) work for every frame. The **{}** button beside each text inserts a field, and the resolved text shows beneath it. Values carry no unit; a value the frame lacks shows as "–", and an unknown name stays as typed (`{{` and `}}` are literal braces). In a movie each frame fills in its own values.

## Movies

**Export → Movie…** opens a window that exports the frames the timeline shows (the funnel filter chooses them) as an H.264 MP4 — a single frame works too, for example one converged model. It needs an `ffmpeg` executable, found on `PATH`, in the usual Homebrew and system locations, or named by `RAYFEM_FFMPEG`; it uses libx264 when available and otherwise the platform's hardware encoder. Nothing is built into RayFEM.

- **Frames:** a range of the shown frames and the video rate (24, 30 or 60 fps). **Per frame** holds each frame for a set time; **Total** sets the movie's length and the frames share it evenly, so one frame can fill the whole movie.
- **Keyframes:** keyframes sit on the movie's own timeline, in seconds. Scrub the preview to a moment (the view shows that moment's frame and scene), arrange the camera, the cut (View → Clipping) and which annotations show, then **Add keyframe here**. Each keyframe can be moved in time, revisited, replaced with the scene on screen or removed. Between keyframes the camera turns along the shortest rotation, zooms geometrically and moves its target in a straight line, and a cut on at both keyframes moves and turns its plane; a cut switching on or off, its mode, and which annotations show change at the next keyframe. An annotation made after a keyframe was recorded keeps its own visibility, so adding one never hides it. Before the first and after the last keyframe the scene holds still; with none, the movie keeps the view it has when the export starts.
- **Easing:** each move from one keyframe to the next has its own easing, chosen in the row beneath the keyframe, next to a curve of its progress over time: **Ease in and out** (the default) starts and stops gently, pausing at both keyframes; **Constant speed** moves at one pace; **Ease in** starts from rest and arrives at constant speed; **Ease out** leaves at constant speed and comes to rest; **Hold, then cut** keeps the keyframe's scene until the next keyframe's moment, then cuts to it. Ease in, constant speed and ease out meet at the same pace, so ease in, constant speed through the middle keyframes, then ease out gives one gliding move with no pauses (when neighbouring moves take about as long and go about as far). **Set every move…** applies a choice to all moves at once: ease in and out, constant speed, or that one continuous move. A new keyframe takes the easing of the keyframe before it.
- **Axes:** **As in view** draws the view's axes (View → Appearance → Axes) when the view shows them; **On** and **Off** always or never draw them. They are scaled to the movie as they look on screen.
- **Spin the view** turns the camera about the axis chosen in the toolbar's Spin menu, a set number of turns over the movie; whole turns loop seamlessly. One frame, a total length and one turn make an orbit.
- **Text and HUD:** **Text size** scales annotation text and the HUD from their default of about 3% of the movie's height. **Show a HUD** puts lines of text in a chosen corner of the movie, on a translucent box; each line can hold the same `{name}` fields as annotations, filled in from each frame, for example `residual = {residual:.3e}`. **+ Field** adds a line for a schema value, and the text as it reads for the frame shown appears beneath. The view previews the HUD while the Movie window is open.
- **Output:** size (rounded down to even numbers), optional 2× supersampling, quality as x264's constant rate factor (18 looks lossless), and the file.

The export renders each frame offscreen as figures do, without the selection, profile guides or reference plane unless it is kept; a cut that moves rebuilds the surface for every frame, so it takes longer. The window stays open while you work, so keyframes can be arranged from the live view; frames being exported stay in the history until the export ends.

## History

The viewer keeps up to 1,000 frames and about 2 GiB of field and mesh data by default (`--history` and `--history-mib`, or the timeline's **History** button). When either limit is reached, frames are evicted by the policy set there:

- **Oldest first**, or **Thin older frames**: the newest half of the capacity stays dense and older frames thin to every nth, then every n²th, and so on, so a long run stays represented end to end.
- **Evict last:** frames tagged with chosen tags, or frames the timeline filter shows, go only after every other frame. For example, with the filter discarding everything but converged and accepted frames, protecting *accepted* makes the history evict converged frames once full; protecting both evicts everything else first.
- **Pinned** frames are never evicted: the pin button beside the filter (or `P` over the viewport) pins the frame on screen, and the timeline marks pinned frames. The frame viewed while paused and frames a movie is exporting also stay.

The History popover reports the frame count, memory estimate, pinned frames and how many have been evicted.

## Exercise the viewer

These commands provide small reproducible streams. Run them with the viewer already listening:

```sh
# Curved quadrilaterals and triangles.
./build-rayfem/examples/exampleClient --geometry quad --order 5 --resolution 5 --frames 180
./build-rayfem/examples/exampleClient --geometry tri --order 4 --resolution 5 --frames 180

# Curved hexahedra and tetrahedra.
./build-rayfem/examples/exampleClient --geometry hex --order 3 --resolution 3 --frames 120
./build-rayfem/examples/exampleClient --geometry tet --order 3 --resolution 3 --frames 120

# Rapid streaming: simulation time still advances by --dt per frame.
./build-rayfem/examples/exampleClient --geometry quad --order 3 --frames 1000 --interval 0 --quiet

# Original NURBS geometry is retained; --order controls the example fields.
./build-rayfem/examples/exampleClient --mesh-file subprojects/mfem/data/disc-nurbs.mesh --order 3 --frames 120
./build-rayfem/examples/exampleClient --mesh-file subprojects/mfem/data/beam-hex-nurbs.mesh --order 3 --frames 120

# Sphere to ellipsoid and back, with one mesh registration.
./build-rayfem/examples/exampleClient --mesh-file subprojects/mfem/data/ball-nurbs.mesh \
  --motion ellipsoid --deformation 0.8 --frames 241 --dt 0.02617993877991494 --fps 30

# Straight geometry and no displacement for analytic derivative checks.
./build-rayfem/examples/exampleClient --geometry hex --curvature 0 --no-displacement --frames 1

# Three material bands for categorical coloring and analysis selection.
./build-rayfem/examples/exampleClient --geometry hex --resolution 3 --materials 3 --frames 120
```

`--fps` and `--interval` are alternative ways to request the send cadence. `--interval 0` sends as quickly as projection, serialization, and transport permit. `--dt` controls simulation time independently. `--motion ellipsoid` starts from the reference geometry, reaches axis factors `(1+A, 1-A/2, 1+A/4)` at simulation time pi, and returns at 2pi (`A` is `--deformation`). With the ball mesh this is a sphere-to-ellipsoid cycle. `--deformation` controls the displacement amplitude, and `--no-displacement` leaves frames on the registered geometry. Imported meshes must be full-dimensional 2D or 3D meshes for this example's ND/RT spaces.

On the undeformed straight geometry, the example's vector fields have these analytic values:

| Field | Definition | Divergence | Curl |
| --- | --- | --- | --- |
| H1 velocity, 2D | `(0.2x-y, x+0.3y)` | `0.5` | `2` |
| H1 velocity, 3D | `(0.2x-y, x+0.3y, 0.4z)` | `0.9` | `(0,0,2)` |
| ND circulation | `(-y,x[,0])` | `0` | `2` in 2D; `(0,0,2)` in 3D |
| RT flux | `(x,y[,z])` | Spatial dimension | `0` |

These are useful checks for the appropriate derivative view. On curved geometry, projection into a chosen finite element space can introduce approximation error. Displacement changes displayed positions only: field values, vector components, and derivatives are evaluated on the registered, undeformed mesh. RayFEM does not transport or recompute ND/RT coefficients on the displayed geometry.

## Moving-mesh diffusion example

`examples/sandbox.cpp` is a small MFEM solver that diffuses a smooth temperature field on an expanding and contracting square. It starts RayFEM automatically, or reuses the compatible viewer already listening:

```sh
./build-rayfem/examples/sandbox
# Run the same calculation and numerical checks without a viewer or pacing.
./build-rayfem/examples/sandbox --no-stream
# Compare diffusion on a fixed mesh.
./build-rayfem/examples/sandbox --no-stream --amplitude 0
```

The example streams one immutable reference mesh, then **Temperature**, **Mesh velocity**, and a displacement for every frame. A separate solver mesh moves according to `s(t) = 1 + amplitude*sin(2*pi*t/period)`. MFEM reassembles mass and diffusion matrices on that geometry and solves `(M_new + dt*K_new)*T_new = M_old*T_old` with backward Euler and conjugate gradients.

This is conservative diffusion, `partial_t T + div(w*T) = diffusivity*Laplacian(T)`, with zero diffusive flux at the moving boundary. Here `T` represents heat per unit area: expansion dilutes it and contraction concentrates it while total heat remains constant. Each step checks the linear residual, total heat, and domain area; fixed-mesh runs also check energy decay. `--dt` controls simulation time, while `--fps` controls streaming pace. Use `--help` for mesh order/resolution, time-step, diffusivity, and pulsation settings.

## Vector flow

Select a vector field, then click **Flow**. **View → Vector flow** offers **Arrows**, **Streamlines**, or both. Arrows sample throughout the domain, independently of the number of elements; a single high-order element can therefore display many arrows. Seeds are spread within elements and across elements, weighted by an estimate of their physical volume. They map directly through the curved geometry, avoiding a costly inverse search for every seed. Arrows have a common adjustable length, with color indicating magnitude. **Through surfaces** keeps interior vectors visible through an opaque mesh. Turn it off to use ordinary surface occlusion.

Streamlines follow the selected frame's finite element vector field using selectable RK2 or RK4 integration. **Vector flow** provides separate **Arrow count** and **Streamline count** controls, requesting a total across the included domain rather than a resolution raised to mesh dimension. The display reports actual/requested counts; zero vectors, clipping, and bounded preview work can produce fewer paths. **Update flow** allows a longer calculation for dense views. **Flow settings** controls step length, maximum path length, integration in both directions, and the moving highlight. A bright, smooth pulse travels along each line, with adjustable pulse length and optional gradual thickening. Cached GPU ribbons animate without rebuilding finite-element samples or drawing moving spheres. These are steady streamlines, not particle trajectories integrated through successive simulation times. The library's `FlowOptions::seed_count` requests a direct total; zero preserves `seed_resolution` behavior, and the existing seed, point, and evaluation budgets still apply.

Automatic previews update at most four times per second with a short computation budget. Pause on a frame and press **Update flow** for a larger budget. The panel reports the sampled frame, arrow/path counts, and whether a preview is incomplete. Material exclusions and either clipping mode apply to flow sampling and display. Switching frames, fields, displacement, or selection invalidates the cached result.

`libRayfem/vector/flow.hpp` exposes `SampleVectorField` and `BuildStreamlines`, with explicit seed positions, RK2/RK4 selection, point/evaluation/search limits, and a cancellation callback. Location and evaluation use the actual curved MFEM mapping, including NURBS and geometry-space displacement. H1 vectors and ND/RT vector bases retain their reference-mesh field interpretation under displacement. Streamline integration uses normalized vectors so step length and path limits are physical distances. Paths stop at the domain or selection boundary, stagnation, invalid values, or a failed point location. Sampling is bounded and approximate: finite seed sets can miss very thin regions, and unusually distorted geometry can require smaller steps or different seeds.

Streamlines currently require full-dimensional 2D or 3D meshes. Arrow sampling also supports 2D surfaces embedded in 3D; streamlines on those surfaces need a separate surface-constrained integrator.

The explicit desktop test `build-rayfem/tests/graphics/streamline_renderer_test build-rayfem/test-output/streamline-pulses` checks moving highlights, joins, thickness, clipping phase, depth occlusion, and near-camera rendering with framebuffer readback. `build-rayfem/tests/graphics/flow_viewer_test build-rayfem/test-output/flow-pulses subprojects/mfem/data/disc-nurbs.mesh subprojects/mfem/data/ball-nurbs.mesh` checks field sampling and rendering together in both themes. These tests require a desktop graphics session; the ordinary Meson suite also checks pulse math and direct seed counts without a window.

## Frame tracking and in-window plots

The compact **View** inspector controls the field, regions, appearance, deformation, and clipping. **Analyze** contains profile and tracking controls; **Info** holds connection and performance details. **Live** follows incoming frames. Playback and the slider inspect retained frames. **Plots** hides or shows the lower plot area.

**View → Appearance → Explode elements** opens gaps between every element, from zero (the intact mesh) to 0.95 (small separated cells). It uniformly shrinks each displayed element about its mapped center after applying displacement, preserving curved and NURBS shapes. Interior faces become visible, including distinct traces of discontinuous fields. Element edges and DOF markers follow the separated elements; shared DOFs retain their global IDs and appear on each owning cell. Clipping operates on the displayed separated geometry. Material exclusions and boundary IDs are preserved; newly exposed internal faces have boundary ID 0. The same setting is available as `RenderOptions::element_explosion` or the viewer's `--explode` option.

Explosion changes display geometry only. Profile coordinates, integrals, tracking, and vector-flow paths continue to use the physical domain. The reference mesh and field coefficients are never modified, and the automatic color range continues to cover the included field volume.

Click a visible element to select it; Shift-click adds or removes elements from the selection. Clicking empty space clears the selection, while dragging still orbits. Selected cells are highlighted in gold. Picking follows the actual curved, displaced, exploded, or clipped surface, including section caps.

The **Elements** tab provides **Hide selected**, **Show selected**, and a scrollable object explorer with visibility controls for every element. Filter the list to selected or hidden cells to restore individual elements, or use **Show all**. IDs are zero-based MFEM element indices. Selection and visibility persist across frames that share the same mesh registration; a different mesh registration or revision resets them. Material exclusions still apply when an element's visibility checkbox is on.

The collapsible **Selection statistics** pane reports physical area/volume, centroid, bounds, materials, geometry/order information, and the current field operation's minimum, maximum, mean, and integral. Bounds and extrema are sampled estimates; integrals use physical quadrature weights. Measurements include displacement and exclude hidden cells and materials, while display clipping and explosion do not alter their physical meaning. On a new streamed frame, use **Update** to refresh the snapshot without repeatedly performing expensive integration during playback. The library API is `ComputeElementStatistics` in `libRayfem/mesh/element_statistics.hpp`.

Hidden cells are omitted from rendering, DOF/vector overlays, field ranges, profiles, surface traces, and tracking. Changing visibility marks profile snapshots stale and rebuilds tracks from retained frames. CSV exports record excluded element IDs. The library equivalents are `RenderOptions::excluded_elements` and scoped `TrackingRequest::excluded_elements`; indices remain those of the original registered mesh.

Surface construction batches element field evaluation and reuses polynomial reference bases within each build, with bounded memory and direct MFEM evaluation for rational NURBS bases. Element outlines are cached in GPU buffers, including their camera-dependent depth offset. Color-map, scale, and range changes update only the color buffer instead of rebuilding the mesh. View caches remain bounded. For repeatable performance measurements, run `build-rayfem/tests/graphics/render_benchmark`; the explicit desktop test `build-rayfem/tests/graphics/edge_renderer_test build-rayfem/test-output` checks cached outlines and compares their CPU submission cost with the former per-edge path. `build-rayfem/tests/integration/viewer_lifecycle_test /absolute/path/to/RayFEM` exercises the real window's startup, reuse, shutdown, and restart on ports 5696–5697.

Click **Pick DOF**, then click a marker on the mesh. RayFEM adds a colored coefficient trace to **Tracks**, backfills the retained history, and samples incoming frames. The selected marker uses its trace color. Alternatively, enter a scalar-space DOF index and coefficient component under **Analyze → Frame tracking**. `DofToVDof` maps vector ordering; ND/RT traces represent oriented functional coefficients, and NURBS traces represent control coefficients. They do not become physical point values merely because their markers move with displacement.

Tracks retain up to 4,096 scalar samples each, with up to eight visible/configured traces. They continue after full frames leave the history. Time/frame axes, hover crosshairs, sample values, and a cursor for the displayed frame are available in the chart. Missing fields, skipped frame indices, and incompatible DOF associations produce gaps. A selected DOF stays bound to its original session, mesh registration, and finite element basis/layout; add a new track after changing topology or basis. Backfill and incoming work share a bounded queue and sampling budget; frames evicted before sampling are counted in the inspector.

**Save CSV** exports a completed trace to `rayfem-exports/track-N.csv`, with frame/time, validity, coefficient/view settings, mesh/session identity, and errors. Files contain the currently retained scalar samples. Removal deletes a trace from the UI only. Point probes, quadrature-sampled extrema, physical mean/integral, and L2 norm are also available. Statistics use the selected field operation and geometry settings at track creation. Integration weights follow the displayed displaced geometry; values and derivatives retain the reference-mesh convention. The exact displacement geometry requirements match profiles. Minimum/maximum are sampled estimates, not guaranteed global finite-element extrema.

`libRayfem/frame/tracking.hpp` exposes `TrackingRequest`, `FrameTracker`, and `ExportTrackingCSV` independently of the UI. Call `bindDof(currentFrame)` before backfilling mixed histories for a picked DOF. Tracks store scalar results and bounded identity metadata, never frame pointers. A track stops after 256 distinct stream/mesh/revision identities; remove it and add another to start fresh.

### Plot settings and export

The gear at the top right of each plot sets its axes: **Linear** or **Log** scale for each (values at or below zero have no place on a log axis and are hidden, with a count), an automatic range or a set one (switching it off starts from the range shown), and labels that replace the plot's own. **Appearance** turns the grid on or off and sets the line width. The gear turns the accent color while an axis is log or set. Settings last for the session, per plot.

**Export** writes the plot as a PNG or a PDF on a white page, with an optional title: a size in inches (3.5, 6.5 and 7 in presets for journal columns), text size in points, and for PNG a resolution. PDF keeps the lines and text as vector graphics. Linear axes tick at round numbers and log axes at whole decades. The raw data stays available through each panel's CSV export.

## Ray and plane profiles

Use **1D profile** or **2D profile** in the viewport toolbar. The **Analyze** inspector holds the sampling controls, and the **Profile** tab below the mesh shows the result. Detailed coordinates and the replaceable sampling method are under **Sampling settings**. Edit the coordinates or drag the gold handles in the viewport, select a sampling algorithm, then press **Sample profile**. The ray plot preserves gaps outside the mesh; the plane heatmap preserves physical aspect ratio and supports value hover. Profiles sample the currently selected field and view operation, independently of the display clipping plane.

Choose a path and press **Export CSV** to save the sampled result. Relative paths are resolved from the viewer's working directory. Changing the frame, field, displacement, profile geometry, or algorithm marks the preview as stale and disables export until it is resampled. Sampling is explicit so it does not interrupt every streamed frame.

`libRayfem/profile/sampler.hpp` provides ray and plane sampling independently of the viewer. Ray samples include both endpoints and use physical distance along a normalized direction. Plane samples use an orthonormal basis with physical `u` and `v` extents, stored row by row with `u` varying fastest. Each returned sample includes its physical position, value, validity, and containing element.

The built-in `exhaustive` and `coherent` algorithms locate points through MFEM inverse element mappings and evaluate the finite element field there. `coherent` uses the preceding sample to accelerate the search. Both choose the lowest element ID when a point lies on a shared boundary, making discontinuous-field traces reproducible. Locations outside the mesh remain in the result with `valid=false`, `element=-1`, and a NaN value; they are not filled with a nearby node's value.

```cpp
#include "libRayfem/profile/sampler.hpp"
#include <fstream>

RayFEM::ProfileRegistry registry;
auto locations = RayFEM::MakeRaySamples({0, 0.5, 0}, {1, 0, 0}, 1.0, 101);
auto samples = registry.sample("coherent", mesh, temperature, locations);

RayFEM::ProfileMetadata metadata;
metadata.field_name = "Temperature";
metadata.frame = step;
metadata.time = step * dt;
metadata.algorithm = "coherent";
std::ofstream csv("temperature-profile.csv");
RayFEM::ExportProfileCSV(csv, samples, metadata);
```

Pass `RenderOptions` to choose magnitude, a component, divergence, or curl using the same field semantics as surface views. For profiles through displaced geometry, the displacement space must match the mesh nodes' finite element collection and degrees of freedom; either MFEM vector ordering is accepted. The sampler locates points in the displayed geometry while evaluating field values and derivatives on the original reference mesh. Other displacement spaces can be displayed but are explicitly unavailable for displaced-profile sampling.

Applications can subclass `ProfileAlgorithm` and call `ProfileRegistry::registerAlgorithm` to add or replace an algorithm by its ID. `algorithms()` supplies the available IDs and display names. CSV exports include the selected field, frame, time, view, component, and algorithm on every row, followed by `s/u/v`, physical coordinates, value, validity, and element ID. Field names are escaped for ordinary CSV readers.

## Surface traces

Choose **Surface trace** to measure a field along a parameterized path on a 3D object's exterior. A **Meridian / colatitude** trace varies the angle from the chosen axis; a **Latitude / azimuth** trace circles that axis at a fixed colatitude. **Trace settings** holds the center, axis, angular range, sample count, refinement, and selectable sampling method. Place the center inside the object for a radial trace, then press **Sample surface**. The gold line follows the sampled surface, and the **Surface** plot displays values against angle or approximate arc length. Hover to inspect values and use **Export → Save CSV** to save the result.

Each path sample casts an independent ray and selects its nearest forward hit on an original exterior face. Hidden materials are skipped; a missed ray stays a gap. Newly exposed internal material interfaces and clipping caps are not surface-trace targets. Volume meshes and 2D surfaces embedded in 3D are supported. The center and axis can be moved in the inspector; azimuth starts at projected world X, or world Y when X is nearly parallel to the axis. Field values and derivatives retain their reference-mesh meaning while hit positions follow displacement.

The default **curved** algorithm refines faces to find initial intersections, then solves the ray intersection against the actual curved finite-element face. The **refined** algorithm uses triangle intersections to choose a reference point and evaluates that point on the actual face; its distance from the ray is exported as a residual. Refinement improves search coverage, but highly folded faces can require more refinement and are not guaranteed to have every intersection discovered. Arc length sums chords between consecutive valid hits without bridging gaps. Increasing the sample count improves this approximation.

`libRayfem/profile/surface_trace.hpp` exposes the same mechanism independently of the UI. `MakeMeridianRays` and `MakeAzimuthRays` create angular paths in radians; arbitrary paths can supply their own ordered `SurfaceTraceRay` list with an application-defined parameter. Subclass `SurfaceTraceAlgorithm` and register it with `SurfaceTraceRegistry::registerAlgorithm` to add or replace a selectable algorithm. Samples and CSV include the parameter, arc distance, ray distance, physical and reference positions, field value, validity, intersection residual, and element/face/boundary/material IDs, plus field/frame/algorithm and material-selection metadata. Requests exceeding 250,000 seed triangles, 24 million ray–triangle checks, or 100,000 curved ray–face solves are rejected with a request to reduce refinement or sampling, keeping synchronous work bounded.

## Surface shape

**Analyze → Surface shape** measures the outer surface's radius from a center about a polar axis, for rotating models' oblateness. The axis defaults to the rotation axis the client reports (see [model axes](streaming.md#model-axes)), otherwise world Z; World X, Y or a custom direction are choices. The center is automatic (the mesh's center, moved onto the model's axis when measuring about it) or set.

Rays go out from the center along several meridians (eight by default, 181 colatitudes each from pole to pole) and around the equator (180 rays), and meet the true curved finite-element faces as the curved surface trace does, so the radii do not depend on the drawn tessellation; displacement is included. The panel reports:

- **Equatorial radius**, the mean radius around the equator, and **polar radius**, the mean of the north and south poles (each shown when they differ);
- **Flattening** f = 1 − R_pol/R_eq and the **axis ratio** R_pol/R_eq;
- **Volume radius**, the radius of a sphere of the same volume, integrated over the meridians;
- **Equator ellipticity**, 1 − shortest/longest equatorial radius, when the surface is not axisymmetric.

**Copy** puts these on the clipboard. The view draws the axis between the poles, the first meridian and the equator. The **Shape** plot shows radius against colatitude (the mean over the meridians, with the smallest and largest when there are several), radius divided by the equatorial radius, or **Flattening over time**: each frame measured while the panel is on adds a point, so streaming a run records its flattening history. **Export** saves the profile, or the history, as CSV. **Sampling** sets the counts.

`libRayfem/profile/surface_shape.hpp` offers the same measurement: `SampleSurfaceShape` traces and measures in one call, splitting the rays into batches that fit the surface trace's per-call limits, and `MakeSurfaceShapeRays` with `MeasureSurfaceShape` separate the steps.

## Materials and boundaries

In **View → Regions**, choose field, material ID, or boundary ID coloring. Material colors follow element attributes; boundary colors follow MFEM boundary attributes. On a 2D mesh, boundary IDs color the element-edge overlay, while area interiors use ID 0. Synthetic exposed material interfaces and cutting sections also use boundary ID 0. Categorical coloring does not change the field used for analysis.

Clear a material checkbox to remove those elements from surfaces, element edges, DOF/vector overlays, ray/plane profiles, surface traces, and tracked measurements. The remaining volume displays newly exposed faces with their own outward orientation and field trace. Material selection is independent of the display clipping plane. Shared DOFs remain available when at least one incident element is included; a DOF supported only by hidden elements yields an invalid tracking sample. Point searches on shared boundaries choose the lowest included element ID.

Changing material selection marks profile snapshots stale. Existing tracks keep their original DOF association but rebuild their results from the full frames still retained by the viewer; older scalar-only history is discarded because its fields are no longer available for reanalysis. CSV exports record the excluded material IDs. The same selection is available through `RenderOptions::excluded_materials` and `TrackingRequest::excluded_materials`; `FrameTracker::setExcludedMaterials` resets samples while preserving a selected DOF's binding. **All** and **None** quickly reset the selection, and `exampleClient --materials N` assigns up to 16 element-attribute bands along X for testing.

## Scope

RayFEM renders a tessellated approximation of curved geometry. Increase the refinement setting when viewing strongly curved or high-order meshes. Volume meshes are displayed through their exterior surfaces, with an interior section added when cutting through elements. The finite element field remains attached to the selected frame when its display mode is changed.

Element boundaries follow the original finite element edges through their curved geometry map. They omit the extra diagonals and subdivision edges used to render a smooth surface. Boundaries on a cutting section follow intersections with original element faces.

The cutting modes follow GLVis's distinction: **cut through elements** adds an interior section with sampled field values; **whole elements** retains only cells whose corners lie behind the plane and displays their newly exposed faces. Cut-through offers two algorithms: **Refined** intersects mapped volume subdivisions and is intended for curved and NURBS geometry; **Planar** intersects original corner edges and is faster for planar-faced meshes. The planar approximation can differ visibly on curved meshes. Refinement improves the curved approximation; cuts retain the half-space where `dot(position - plane.point, plane.normal) <= 0`. The CPU APIs are in `libRayfem/mesh/clipping.hpp`.

The viewer retains at most 1,000 frames by default, also bounded by a 2 GiB retention estimate covering field coefficients, finite-element bookkeeping, and distinct shared meshes (see [History](#history) for limits and eviction). Shared geometry is counted once. This estimate is not an exact allocator or total process-memory limit, and the viewer keeps at least one complete frame. Its receive queue holds at most 64 complete frames or 128 MiB and drops the oldest complete frames when overloaded. The status panel reports these drops. Mesh registration storage is limited to 16 meshes or 128 MiB; re-register a mesh if its registration is evicted. Each frame supports up to 64 unique named fields, with names up to 255 bytes, and each packet has a 64 MiB payload limit. Rapid streams can outrun rendering: a displayed frame rate is distinct from the producer's send rate. For persistent storage or exact replay of an entire long run, keep simulation output separately.

The streaming transport is intended for trusted simulations. It is not an authenticated public service. The example uses serial MFEM meshes and grid functions; distributed parallel mesh assembly is outside this example's scope.

The receiver accepts polynomial field orders up to 32 and NURBS degrees up to 10. The NURBS bound protects the fixed-size evaluator in the bundled MFEM version; higher degrees are rejected before evaluation.
