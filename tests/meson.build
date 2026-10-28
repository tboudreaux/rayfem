geometry_test = executable('geometry_test', 'geometry_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('geometry', geometry_test, args: [meson.project_source_root() / 'subprojects/mfem/data'], timeout: 120)

field_range_test = executable('field_range_test', 'field_range_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('field_range', field_range_test,
    args: [meson.project_source_root() / 'subprojects/mfem/data'], timeout: 120)

streaming_test = executable('streaming_test', 'streaming_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('streaming', streaming_test,
    args: [meson.project_source_root() / 'subprojects/mfem/data/square-nurbs.mesh'], timeout: 120)

launcher_test = executable('launcher_test', 'launcher_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('launcher', launcher_test, timeout: 30)

shutdown_test = executable('shutdown_test', 'shutdown_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('shutdown', shutdown_test, timeout: 30)

serif_test = executable('serif_test', 'serif_test.cpp',
    dependencies: [rayfem_dep, serif_dep],
    cpp_args: cpp_args + ['-DRAYFEM_HAS_SERIF=' + (serif_enabled ? '1' : '0')],
    override_options: ['cpp_std=' + (serif_enabled ? 'c++26' : 'c++23')],
    build_rpath: ':'.join(serif_runtime_dirs), link_args: link_args)
test('serif_reader', serif_test, timeout: 120)

profile_test = executable('profile_test', 'profile_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('profiles', profile_test, args: [meson.project_source_root() / 'subprojects/mfem/data'], timeout: 120)

clipping_test = executable('clipping_test', 'clipping_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('clipping', clipping_test, args: [meson.project_source_root() / 'subprojects/mfem/data'], timeout: 120)

contours_test = executable('contours_test', 'contours_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('contours', contours_test, timeout: 60)

# Requires a desktop graphics session; run explicitly (see README).
executable('viewer_test', 'viewer_test.cpp', dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
executable('curvature_viewer_test', 'curvature_viewer_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)

camera_test = executable('camera_controller_test', 'camera_controller_test.cpp',
    dependencies: raylib_dep)
test('camera_controller', camera_test, timeout: 30)

reference_plane_test = executable('reference_plane_test', 'reference_plane_test.cpp',
    dependencies: raylib_dep)
test('reference_plane', reference_plane_test, timeout: 30)

orientation_cube_test = executable('orientation_cube_test', 'orientation_cube_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('orientation_cube', orientation_cube_test, timeout: 30)

color_map_test = executable('color_map_test', 'color_map_test.cpp',
    dependencies: rayfem_dep)
test('color_maps', color_map_test, timeout: 30)

color_scale_test = executable('color_scale_test', 'color_scale_test.cpp',
    dependencies: rayfem_dep)
test('color_scales', color_scale_test, timeout: 30)

displacement_test = executable('displacement_test', 'displacement_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('displacement', displacement_test,
    args: [meson.project_source_root() / 'subprojects/mfem/data/ball-nurbs.mesh'], timeout: 120)

tracking_test = executable('tracking_test', 'tracking_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('tracking', tracking_test,
    args: [meson.project_source_root() / 'subprojects/mfem/data'], timeout: 120)

plot_test = executable('plot_test', 'plot_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('plot_helpers', plot_test, timeout: 30)

flow_test = executable('flow_test', 'flow_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('flow', flow_test,
    args: [meson.project_source_root() / 'subprojects/mfem/data'], timeout: 120)

# Explicit desktop/external-fixture checks, not part of the default CPU suite.
executable('flow_viewer_test', 'flow_viewer_test.cpp', dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
executable('stroid_flow_test', 'stroid_flow_test.cpp', dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
executable('imgui_backend_test', 'imgui_backend_test.cpp', dependencies: rlimgui_dep, link_args: link_args)
executable('edge_renderer_test', 'edge_renderer_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
executable('viewer_lifecycle_test', 'viewer_lifecycle_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
executable('render_benchmark', 'render_benchmark.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)

explosion_test = executable('explosion_test', 'explosion_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('explosion', explosion_test,
    args: [meson.project_source_root() / 'subprojects/mfem/data'], timeout: 120)

surface_trace_test = executable('surface_trace_test', 'surface_trace_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('surface_traces', surface_trace_test,
    args: [meson.project_source_root() / 'subprojects/mfem/data'], timeout: 120)

selection_test = executable('selection_test', 'selection_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('selection', selection_test,
    args: [meson.project_source_root() / 'subprojects/mfem/data'], timeout: 120)

element_visibility_test = executable('element_visibility_test', 'element_visibility_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('element_visibility', element_visibility_test, timeout: 120)

ray_placement_test = executable('ray_placement_test', 'ray_placement_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('ray_placement', ray_placement_test, timeout: 60)

frame_pipeline_test = executable('frame_pipeline_test', 'frame_pipeline_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('frame_pipeline', frame_pipeline_test, timeout: 120)

element_picker_test = executable('element_picker_test', 'element_picker_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('element_picker', element_picker_test,
    args: [meson.project_source_root() / 'subprojects/mfem/data'], timeout: 120)

element_statistics_test = executable('element_statistics_test', 'element_statistics_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('element_statistics', element_statistics_test,
    args: [meson.project_source_root() / 'subprojects/mfem/data'], timeout: 120)

element_panel_test = executable('element_panel_test', 'element_panel_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('element_panel', element_panel_test, timeout: 30)

streamline_pulse_test = executable('streamline_pulse_test', 'streamline_pulse_test.cpp',
    dependencies: raylib_dep)
test('streamline_pulse', streamline_pulse_test, timeout: 30)

# Requires a desktop OpenGL context; run explicitly alongside flow_viewer_test.
executable('streamline_renderer_test', 'streamline_renderer_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)

# The real example checks heat conservation and moving-domain area each step,
# plus L2-energy decay when the mesh is fixed. No network or window required.
if build_examples
    test('sandbox_pulsating', sandbox,
    args: ['--no-stream', '--resolution', '5', '--steps', '40',
           '--dt', '0.05', '--period', '1', '--amplitude', '0.25'], timeout: 60)
test('sandbox_fixed', sandbox,
    args: ['--no-stream', '--resolution', '5', '--steps', '40',
           '--dt', '0.05', '--amplitude', '0'], timeout: 60)

endif
