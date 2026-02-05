cmake = import('cmake')
mfem_cmake_options = cmake.subproject_options()
mfem_cmake_options.add_cmake_defines({
    'MFEM_ENABLE_EXAMPLES': 'OFF',
    'MFEM_ENABLE_TESTING': 'OFF',
    'MFEM_ENABLE_MINIAPPS': 'OFF',
    'MFEM_USE_BENCMARK': 'OFF',
    'BUILD_SHARED_LIBS': 'OFF',
    'BUILD_STATIC_LIBS': 'ON',
})
mfem_cmake_options.set_install(true)

mfem_sp = cmake.subproject(
    'mfem',
    options: mfem_cmake_options)
dependencies += mfem_sp.dependency('mfem')
