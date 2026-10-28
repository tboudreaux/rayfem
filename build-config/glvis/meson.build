sdl2_dep = dependency('sdl2')
glew_dep = dependency('glew')
freetype_dep = dependency('freetype2')
fontconfig_dep = dependency('fontconfig')

glm_dep = dependency('glm', required : false)

mfem_rel_path = '../../mfem/__CMake_build'
glvis_cmake_options = cmake.subproject_options()
glvis_cmake_options.add_cmake_defines({
  'GLVIS_USE_LIBPNG': 'ON',
  'GLVIS_USE_LIBTIFF': 'OFF',
  'GLVIS_MULTISAMPLE': '4',
})

glvis_sp = cmake.subproject('glvis', options: glvis_cmake_options)

glvis_lib_dep = glvis_sp.dependency('glvis')
dependencies += [sdl2_dep, glew_dep, freetype_dep, fontconfig_dep, glm_dep, glvis_lib_dep]