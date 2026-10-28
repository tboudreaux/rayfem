raylib_opts = cmake.subproject_options()
raylib_opts.set_install(true)
raylib_opts.add_cmake_defines({
      'BUILD_EXAMPLES': 'OFF',
      'BUILD_SHARED_LIBS': 'ON',
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
raylib_dep = raylib_subproject.dependency('raylib')
dependencies += raylib_dep
raylib_header = fs.read(meson.project_source_root() / 'subprojects/raylib/src/raylib.h')
raylib_version = raylib_header.split('#define RAYLIB_VERSION ')[1].split('\n')[0].strip().strip('"')
pkg.generate(raylib_subproject.target('raylib'), name: 'RayFEM raylib',
             filebase: 'rayfem-raylib', version: raylib_version, subdirs: '.')

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
