include_dirs = include_directories('include')
sources = files(
    'lib/network/client.cpp',
    'lib/network/server.cpp',
    'lib/mesh/converter.cpp'
)

libRayFem = library(
    'libRayFem',
    sources,
    include_directories: include_dirs,
    dependencies: dependencies,
    cpp_args: cpp_args,
    install: true
)

rayfem_dep = declare_dependency(
    include_directories: include_dirs,
    link_with: libRayFem,
    sources: sources,
    dependencies: dependencies,
    compile_args: cpp_args,
)