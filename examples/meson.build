# Example producers: exampleClient streams synthetic meshes and fields;
# sandbox solves a small diffusion problem and starts the viewer itself.
executable('exampleClient', 'client.cpp',
    dependencies: [app_dependencies, rayfem_dep],
    cpp_args: cpp_args, c_args: c_args, link_args: link_args)

sandbox = executable('sandbox', 'sandbox.cpp',
    dependencies: [app_dependencies, rayfem_dep],
    cpp_args: cpp_args, c_args: c_args, link_args: link_args)
