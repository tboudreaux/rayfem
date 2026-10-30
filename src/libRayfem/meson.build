include_dirs = include_directories('include')
# The MFEM-free core: viewer launch and shutdown, and streaming of serialized
# meshes and fields. The Python extension links only this.
client_sources = files('lib/init.cpp', 'lib/network/stream.cpp')
rayfem_client = static_library('rayfem_client', client_sources,
    include_directories: include_dirs, dependencies: [czmq_dep, zmq_dep], cpp_args: cpp_args, pic: true)
rayfem_client_dep = declare_dependency(include_directories: include_dirs, link_with: rayfem_client,
    dependencies: [czmq_dep, zmq_dep])
sources = files(
    'lib/mfem_compat.cpp',
    'lib/threading.cpp',
    'lib/network/client.cpp',
    'lib/network/server.cpp',
    'lib/mesh/converter.cpp',
    'lib/mesh/explosion.cpp',
    'lib/mesh/clipping.cpp',
    'lib/mesh/field_range.cpp',
    'lib/mesh/element_statistics.cpp',
    'lib/mesh/contours.cpp',
    'lib/profile/sampler.cpp',
    'lib/profile/surface_trace.cpp',
    'lib/frame/tracking.cpp',
    'lib/vector/flow.cpp'
)

serif_adapter = static_library('rayfem_serif_adapter', 'lib/io/serif.cpp',
    include_directories: include_dirs, dependencies: [mfem_dep, serif_dep],
    cpp_args: cpp_args + ['-DRAYFEM_HAS_SERIF=' + (serif_enabled ? '1' : '0')],
    override_options: ['cpp_std=' + (serif_enabled ? 'c++26' : 'c++23')], pic: true)

libRayFem = library(
    'rayfem',
    sources,
    include_directories: include_dirs,
    dependencies: dependencies,
    cpp_args: cpp_args,
    link_whole: [serif_adapter, rayfem_client],
    build_rpath: ':'.join(serif_runtime_dirs + runtime_dirs),
    install_rpath: ':'.join(serif_runtime_dirs + runtime_dirs + [get_option('prefix') / get_option('libdir')]),
    install: not bundle
)

rayfem_dep = declare_dependency(
    include_directories: include_dirs,
    link_with: libRayFem,
    dependencies: dependencies,
    compile_args: cpp_args,
)
meson.override_dependency('librayfem', rayfem_dep)
meson.override_dependency('rayfem', rayfem_dep)

if not bundle
    install_subdir(
      'include',
      install_dir: get_option('includedir'),
      strip_directory: true
    )
endif
