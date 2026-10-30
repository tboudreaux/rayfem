# A parent's override is the authoritative ABI. Never instantiate a second
# MFEM subproject when the caller already supplies one.
mfem_prefix = get_option('mfem_prefix')
if mfem_prefix != '' and not fs.is_absolute(mfem_prefix)
    mfem_prefix = meson.project_source_root() / mfem_prefix
endif
if serif_enabled and serif_mfem_prefix != ''
    if mfem_prefix != '' and not fs.is_samepath(mfem_prefix, serif_mfem_prefix)
        error('SERiF and RayFEM must use the same MFEM prefix: ' + serif_mfem_prefix)
    endif
    mfem_prefix = serif_mfem_prefix
endif

mfem_dep = disabler()
mfem_provider = ''
register_mfem_override = false
if bundle and get_option('mfem_provider') == 'external'
    error('-Dbundle=true builds its own static MFEM; drop mfem_provider=external')
endif
if not bundle and (meson.is_subproject() or (mfem_prefix == '' and get_option('mfem_provider') == 'external'))
    mfem_dep = dependency('mfem', required: false, allow_fallback: false)
    if mfem_dep.found()
        mfem_provider = 'parent or installed dependency'
        if mfem_prefix != ''
            parent_prefix = mfem_dep.get_variable(internal: 'prefix', pkgconfig: 'prefix', default_value: '')
            if parent_prefix == '' or not fs.is_samepath(parent_prefix, mfem_prefix)
                error('The parent MFEM dependency must expose its matching SERiF prefix through variable prefix.')
            endif
        endif
    endif
endif

if not bundle and not mfem_dep.found() and get_option('mfem_provider') != 'bundled'
    mfem_cmake_args = []
    if mfem_prefix != ''
        mfem_cmake_args = ['-DMFEM_DIR=' + mfem_prefix / 'lib/cmake/mfem']
    endif
    mfem_dep = dependency('MFEM', method: 'cmake', modules: ['mfem'],
                          cmake_args: mfem_cmake_args, required: mfem_prefix != '', allow_fallback: false)
    if mfem_dep.found() and not cxx.compiles('''
        #include <mfem.hpp>
        #include <type_traits>
        #ifndef MFEM_USE_EXCEPTIONS
        #error RayFEM requires exception-enabled MFEM
        #endif
        static_assert(std::is_same_v<mfem::real_t,double>);
        int main() { return 0; }
        ''', dependencies: mfem_dep, name: 'MFEM supports RayFEM exception/double ABI')
        if mfem_prefix != '' or get_option('mfem_provider') == 'external'
            error('External MFEM must enable exceptions and double precision with usable public headers.')
        endif
        warning('Ignoring an incompatible installed MFEM; using the bundled exception-enabled build.')
        mfem_dep = disabler()
    endif
    if mfem_dep.found()
        mfem_provider = 'existing CMake installation'
        register_mfem_override = true
    endif
endif

if not mfem_dep.found()
    if serif_enabled
        error('SERiF archive support requires its existing MFEM dependency. Override mfem in the parent or provide the matching mfem_prefix; a second bundled ABI is not allowed.')
    endif
    if get_option('mfem_provider') == 'external'
        error('No external MFEM was found. Supply a parent override, mfem_prefix, or an installed MFEM package.')
    endif
    mfem_cmake_options = cmake.subproject_options()
    mfem_cmake_options.add_cmake_defines({
        'MFEM_ENABLE_EXAMPLES': 'OFF', 'MFEM_ENABLE_TESTING': 'OFF',
        'MFEM_ENABLE_MINIAPPS': 'OFF', 'MFEM_USE_BENCHMARK': 'OFF',
        'BUILD_SHARED_LIBS': bundle ? 'OFF' : 'ON', 'MFEM_USE_EXCEPTIONS': 'ON',
        'CMAKE_POSITION_INDEPENDENT_CODE': 'ON',
    })
    mfem_cmake_options.append_compile_args('cpp', cpp_args)
    mfem_cmake_options.set_install(not bundle)
    mfem_sp = cmake.subproject('mfem', options: mfem_cmake_options)
    mfem_dep = mfem_sp.dependency('mfem')
    register_mfem_override = true
    mfem_provider = bundle ? 'bundled static MFEM' : 'bundled shared MFEM'
    mfem_cmake_file = fs.read(meson.project_source_root() / 'subprojects/mfem/CMakeLists.txt')
    mfem_version = mfem_cmake_file.split('set(${PROJECT_NAME}_VERSION ')[1].split(')')[0].strip().strip('"')
    if not bundle
        pkg.generate(mfem_sp.target('mfem'), name: 'RayFEM bundled MFEM',
                     filebase: 'rayfem-mfem', version: mfem_version, subdirs: '.')
    endif
    mfem_build_dir = meson.project_build_root() / 'subprojects/mfem/__CMake_build'
endif

mfem_pkg_dep = mfem_dep
mfem_exported_flags = []
if not mfem_provider.startswith('bundled')
    mfem_runtime_prefix = mfem_prefix
    if mfem_runtime_prefix == ''
        mfem_runtime_prefix = mfem_dep.get_variable(internal: 'prefix', pkgconfig: 'prefix', default_value: '')
    endif
    mfem_runtime_dir = mfem_dep.get_variable(internal: 'libdir', pkgconfig: 'libdir',
                                            cmake: 'MFEM_LIBRARY_DIR', default_value: '')
    if mfem_runtime_dir == ''
        if mfem_runtime_prefix != ''
            mfem_runtime_dir = mfem_runtime_prefix / 'lib'
        endif
    endif
    if mfem_runtime_dir != ''
        runtime_dirs += [mfem_runtime_dir]
    endif

    # MFEM's CMake package exports additional required compiler flags in this
    # variable rather than on the imported target (notably OpenMP). Preserve
    # argument grouping instead of assuming whitespace-separated flag tokens.
    # Native parent dependencies already carry their own compile_args; parents
    # wrapping a CMake dependency can expose the same variable as metadata.
    mfem_flag_string = mfem_dep.get_variable(internal: 'MFEM_CXX_FLAGS',
        pkgconfig: 'MFEM_CXX_FLAGS', cmake: 'MFEM_CXX_FLAGS', default_value: '')
    if mfem_flag_string.strip() != ''
        mfem_python = import('python').find_installation()
        mfem_flag_result = run_command(mfem_python, '-c',
            'import shlex,sys; print(chr(10).join(shlex.split(sys.argv[1])))',
            mfem_flag_string, check: true)
        mfem_exported_flags = mfem_flag_result.stdout().strip().split('\n')
        mfem_dep = declare_dependency(dependencies: mfem_dep, compile_args: mfem_exported_flags,
            variables: {'prefix': mfem_runtime_prefix, 'libdir': mfem_runtime_dir,
                        'MFEM_CXX_FLAGS': mfem_flag_string})
    endif
endif
if register_mfem_override
    meson.override_dependency('mfem', mfem_dep)
endif
dependencies += mfem_dep
