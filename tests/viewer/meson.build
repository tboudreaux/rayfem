# The viewer's own headers (src/viewer), tested without a window.

figure_test = executable('figure_test', 'figure_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('figure', figure_test, timeout: 120)

camera_test = executable('camera_controller_test', 'camera_controller_test.cpp',
    dependencies: [raylib_dep, viewer_headers_dep])
test('camera_controller', camera_test, timeout: 30)

reference_plane_test = executable('reference_plane_test', 'reference_plane_test.cpp',
    dependencies: [raylib_dep, viewer_headers_dep])
test('reference_plane', reference_plane_test, timeout: 30)

orientation_cube_test = executable('orientation_cube_test', 'orientation_cube_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('orientation_cube', orientation_cube_test, timeout: 30)

plot_test = executable('plot_test', 'plot_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('plot_helpers', plot_test, timeout: 30)

ray_placement_test = executable('ray_placement_test', 'ray_placement_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('ray_placement', ray_placement_test, timeout: 60)

frame_pipeline_test = executable('frame_pipeline_test', 'frame_pipeline_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('frame_pipeline', frame_pipeline_test, timeout: 120)

listener_test = executable('listener_test', 'listener_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('listener', listener_test, timeout: 60)

element_picker_test = executable('element_picker_test', 'element_picker_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('element_picker', element_picker_test,
    args: [meson.project_source_root() / 'subprojects/mfem/data'], timeout: 120)

element_panel_test = executable('element_panel_test', 'element_panel_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('element_panel', element_panel_test, timeout: 30)

streamline_pulse_test = executable('streamline_pulse_test', 'streamline_pulse_test.cpp',
    dependencies: [raylib_dep, viewer_headers_dep])
test('streamline_pulse', streamline_pulse_test, timeout: 30)

spin_test = executable('spin_test', 'spin_test.cpp',
    dependencies: [raylib_dep, viewer_headers_dep])
test('spin', spin_test, timeout: 30)

movie_test = executable('movie_test', 'movie_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('movie', movie_test, timeout: 60)

retention_test = executable('retention_test', 'retention_test.cpp', dependencies: viewer_headers_dep)
test('retention', retention_test, timeout: 30)

widgets_test = executable('widgets_test', 'widgets_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('widgets', widgets_test, timeout: 30)
