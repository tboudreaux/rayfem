# Need a desktop graphics session: built, but not run by meson test (see README).

executable('viewer_test', 'viewer_test.cpp', dependencies: [rayfem_dep, app_dependencies], link_args: link_args)

executable('curvature_viewer_test', 'curvature_viewer_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)

executable('flow_viewer_test', 'flow_viewer_test.cpp', dependencies: [rayfem_dep, app_dependencies], link_args: link_args)

executable('imgui_backend_test', 'imgui_backend_test.cpp', dependencies: rlimgui_dep, link_args: link_args)

executable('edge_renderer_test', 'edge_renderer_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)

executable('render_benchmark', 'render_benchmark.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)

executable('streamline_renderer_test', 'streamline_renderer_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
