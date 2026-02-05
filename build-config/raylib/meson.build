raylib_opts = cmake.subproject_options()
raylib_opts.set_install(false)
raylib_opts.add_cmake_defines({
      'BUILD_EXAMPLES': 'OFF',
  })
if cc.get_id() == 'emscripten'
    raylib_opts.add_cmake_defines({
          'PLATFORM': 'Web',
    })
else
    raylib_opts.add_cmake_defines({
        'PLATFORM': 'Desktop',
      })
endif
raylib_subproject = cmake.subproject('raylib', options: raylib_opts)
dependencies += raylib_subproject.dependency('raylib')

# General configuration
if host_machine.system() == 'windows'
    dependencies += [
        cc.find_library('winmm'),
    ]
elif host_machine.system() == 'darwin'
    link_args += [
        '-framework', 'AppKit',
        '-framework', 'IOKit',
    ]
elif host_machine.system() == 'linux'
    dependencies += [
        cc.find_library('m'),
        cc.find_library('dl'),
    ]
elif host_machine.system() == 'emscripten'
    link_args += [
        '-s', 'ENVIRONMENT=web',
        '-s', 'USE_GLFW=3',
    ]
    name_suffix = 'html'
endif