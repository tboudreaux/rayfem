czmq_opts = cmake.subproject_options()
czmq_opts.add_cmake_defines({
    'CMAKE_POLICY_VERSION_MINIMUM': '3.5',
    'CMAKE_C_FLAGS': '-U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0',
    'CMAKE_CXX_FLAGS': '-U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0'
})

czmq_opts.set_install(false)

czmq_subproject = cmake.subproject('czmq', options: czmq_opts)

dependencies += czmq_subproject.dependency('czmq')