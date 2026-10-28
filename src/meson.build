subdir('libRayfem')

viewer_install_rpath = ':'.join(serif_runtime_dirs + runtime_dirs + [get_option('prefix') / get_option('libdir')])
if build_viewer
    rotation_generator = files('../tools/generate_rotation_icons.py')
    rotation_fallback = files('app/rotation_icons_data.hpp')
    rotation_assets_dir = meson.current_source_dir() / '../assets/rotation'
    rotation_sources = files(
        '../assets/rotation/large_clockwise.svg',
        '../assets/rotation/small_clockwise.svg',
        '../assets/rotation/small_counter-clockwise.svg',
        '../assets/rotation/large_counter-clockwise.svg')
    rotation_converter = find_program('rsvg-convert', required: false)
    rotation_args = ['--assets-dir', rotation_assets_dir]
    if rotation_converter.found()
        rotation_args += ['--converter', rotation_converter.full_path()]
    else
        # Fail during configuration as well as incremental builds if changed
        # SVGs cannot be rendered and the portable checked-in fallback is stale.
        run_command(python_build, rotation_generator,
            '--verify-fallback', rotation_fallback,
            '--assets-dir', rotation_assets_dir, check: true)
        rotation_args += ['--fallback', rotation_fallback]
    endif
    rotation_artwork = custom_target('rotation_artwork',
        input: rotation_sources, output: 'rayfem_rotation_icons_generated.hpp',
        command: [python_build, rotation_generator, '--output', '@OUTPUT@'] + rotation_args,
        depend_files: [rotation_generator, rotation_fallback])
    viewer_sources = ['app/main.cpp', rotation_artwork]
    viewer_objcpp_args = []
    if host_machine.system() == 'darwin'
        add_languages('objcpp', required: true)
        viewer_objcpp_args = ['-std=c++23']
        viewer_sources += files('app/mouse_modifiers.mm')
    endif
    executable('RayFEM', viewer_sources,
        dependencies: [app_dependencies, rayfem_dep],
        cpp_args: cpp_args, c_args: c_args, objcpp_args: viewer_objcpp_args, link_args: link_args,
        install: true, install_rpath: viewer_install_rpath)
endif

if build_examples
    executable('exampleClient', 'app/client.cpp',
        dependencies: [app_dependencies, rayfem_dep],
        cpp_args: cpp_args, c_args: c_args, link_args: link_args)

    sandbox = executable('sandbox', 'app/sandbox.cpp',
        dependencies: [app_dependencies, rayfem_dep],
        cpp_args: cpp_args, c_args: c_args, link_args: link_args)
endif

if get_option('glvis_experiment')
    executable('glvisInt', 'app/demo.cpp',
        dependencies: [dependencies, app_dependencies],
        cpp_args: cpp_args, c_args: c_args, link_args: link_args)
endif
