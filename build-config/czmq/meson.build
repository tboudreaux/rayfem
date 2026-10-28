zmq_dep = dependency('libzmq')
dependencies += zmq_dep
czmq_opts = cmake.subproject_options()

zmq_include_dir = zmq_dep.get_variable(pkgconfig: 'includedir', default_value: '')
zmq_library_dir = zmq_dep.get_variable(pkgconfig: 'libdir', default_value: '')
zmq_pkgconfig_dir = zmq_dep.get_variable(pkgconfig: 'pcfiledir', default_value: '')
# CZMQ invokes pkg-config independently through CMake. On reconfiguration,
# preserve the installation Meson selected instead of mixing another libzmq's
# draft-API compile flags with the previously selected headers and library.
if zmq_pkgconfig_dir != ''
    zmq_cmake_configuration = configuration_data()
    zmq_cmake_configuration.set('ZMQ_PKGCONFIG_DIR', zmq_pkgconfig_dir)
    zmq_cmake_configuration.set('PATH_SEPARATOR', host_machine.system() == 'windows' ? ';' : ':')
    zmq_cmake_environment = configure_file(input: 'pkgconfig-environment.cmake.in',
        output: 'pkgconfig-environment.cmake', configuration: zmq_cmake_configuration)
    czmq_opts.add_cmake_defines({'CMAKE_PROJECT_czmq_INCLUDE': zmq_cmake_environment.full_path()})
endif
if zmq_library_dir != '' and host_machine.system() != 'windows'
    runtime_dirs += [zmq_library_dir]
    czmq_opts.append_link_args('-Wl,-rpath,' + zmq_library_dir)
endif

if zmq_include_dir != ''
    czmq_opts.append_compile_args('c', '-I' + zmq_include_dir)
endif

czmq_opts.add_cmake_defines({
    'CMAKE_POLICY_VERSION_MINIMUM': '3.5',
    'CMAKE_C_FLAGS': '-U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0',
    'CMAKE_CXX_FLAGS': '-U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0'
})

czmq_opts.set_install(true)

czmq_subproject = cmake.subproject('czmq', options: czmq_opts)

czmq_dep = czmq_subproject.dependency('czmq')
dependencies += czmq_dep
czmq_version_parts = []
czmq_header = fs.read(meson.project_source_root() / 'subprojects/czmq/include/czmq_library.h')
foreach part : ['MAJOR', 'MINOR', 'PATCH']
    czmq_version_parts += czmq_header.split('#define CZMQ_VERSION_' + part + ' ')[1].split('\n')[0].strip()
endforeach
pkg.generate(czmq_subproject.target('czmq'), name: 'RayFEM CZMQ',
             filebase: 'rayfem-czmq', version: '.'.join(czmq_version_parts),
             subdirs: '.', libraries: zmq_dep)
