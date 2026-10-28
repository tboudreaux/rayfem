# Raylib provides the window/input backend. Auto-discovered GLFW backends
# would otherwise load a second copy of GLFW on macOS.
imgui_options = ['auto_features=disabled', 'default_library=static']
foreach backend : ['dx9', 'dx10', 'dx11', 'dx12', 'metal', 'sdl2_renderer',
                   'sdl3_renderer', 'sdl3_gpu', 'vulkan', 'webgpu', 'glfw',
                   'sdl3', 'osx', 'win', 'allegro5']
    imgui_options += [backend + '=disabled']
endforeach
if get_option('glvis_experiment')
    imgui_options += ['opengl=enabled', 'sdl2=enabled']
else
    imgui_options += ['opengl=disabled', 'sdl2=disabled']
endif
imgui_project = subproject('imgui-docking', default_options: imgui_options)
app_dependencies += imgui_project.get_variable('imgui_dep')
