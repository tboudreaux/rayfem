subdir('libRayFem')
executable(
    'RayFEM',
    'app/main.cpp',
    dependencies: [dependencies, rayfem_dep],
    cpp_args: cpp_args,
    c_args: c_args,
    link_args: link_args
)
executable(
    'exampleClient',
    'app/client.cpp',
    dependencies: [dependencies, rayfem_dep],
    cpp_args: cpp_args,
    c_args: c_args,
    link_args: link_args
)
