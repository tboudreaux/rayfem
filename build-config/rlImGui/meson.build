rlImGuideps = [
    raylib_dep,
    dependency('imgui-docking'),
]

librlimgui = static_library(
  'rlImGui',
  sources: files(
    'rlImGui.cpp',
  ),
  include_directories: include_directories('.'),
  dependencies: rlImGuideps,
)

rlimgui_dep = declare_dependency(
  link_with: librlimgui,
  include_directories: include_directories('.'),
  dependencies: rlImGuideps,
)

app_dependencies += rlimgui_dep
