# rlImGui and its FontAwesome data are vendored in third_party/rlImGui.
rlimgui_source = '../../third_party/rlImGui'
rlImGuideps = [
    raylib_dep,
    dependency('imgui-docking'),
]

librlimgui = static_library(
  'rlImGui',
  sources: files(
    rlimgui_source / 'rlImGui.cpp',
  ),
  include_directories: include_directories(rlimgui_source),
  dependencies: rlImGuideps,
)

rlimgui_dep = declare_dependency(
  link_with: librlimgui,
  include_directories: include_directories(rlimgui_source),
  dependencies: rlImGuideps,
)

app_dependencies += rlimgui_dep
