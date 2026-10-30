project('RayFEM', ['cpp', 'c'],
        version : '0.1.0',
        meson_version : '>=1.1',
        default_options : ['cpp_std=c++23', 'default_library=shared'])

c_args = []
cpp_args = []
link_args = []
name_suffix = []
dependencies = []
app_dependencies = []
runtime_dirs = []
fs = import('fs')
pkg = import('pkgconfig')
build_viewer = get_option('build_viewer').enabled() or (get_option('build_viewer').auto() and not meson.is_subproject())
build_examples = get_option('build_examples').enabled() or (get_option('build_examples').auto() and not meson.is_subproject())
build_tests = get_option('build_tests').enabled() or (get_option('build_tests').auto() and not meson.is_subproject())
build_ui = build_viewer or build_tests or get_option('glvis_experiment')
# A wheel build links everything statically and installs only the Python
# package (extension and viewer), so the wheel runs without other libraries.
bundle = get_option('bundle')
if bundle and get_option('default_library') != 'static'
    error('-Dbundle=true needs -Ddefault_library=static so the viewer and extension are self-contained')
endif

add_project_arguments('-U_FORTIFY_SOURCE', '-D_FORTIFY_SOURCE=0', '-Wno-unused-parameter', language: 'cpp')

cc = meson.get_compiler('c')
cxx = meson.get_compiler('cpp')

# Older Meson releases infer libc++ features from AppleClang's unrelated
# version number and inject a macro removed by recent Apple SDKs.
if host_machine.system() == 'darwin' and cxx.get_id() == 'clang'
    cpp_args += ['-U_LIBCPP_ENABLE_ASSERTIONS', '-D_LIBCPP_HARDENING_MODE=_LIBCPP_HARDENING_MODE_FAST']
    if not meson.is_subproject()
        add_global_arguments(cpp_args, language: 'cpp')
    else
        add_project_arguments(cpp_args, language: 'cpp')
    endif
endif

subdir('build-config')
python_build = import('python').find_installation(pure: false)
if not bundle
    header_install_args = [
        '--includedir', get_option('includedir'),
        '--raylib-build', fs.parent(raylib_subproject.target('raylib').full_path()) / '__CMake_build',
        '--czmq-build', fs.parent(czmq_subproject.target('czmq').full_path()) / '__CMake_build',
    ]
    if mfem_provider == 'bundled shared MFEM'
        header_install_args += ['--mfem-build', fs.parent(mfem_sp.target('mfem').full_path()) / '__CMake_build']
    endif
endif
if not bundle
    meson.add_install_script(python_build, files('tools/install_dependency_headers.py'), header_install_args)
endif
subdir('src')
if build_examples
    subdir('examples')
endif
if get_option('python').enabled()
    subdir('python')
endif
if build_tests
    subdir('tests')
endif

package_rpaths = []
if host_machine.system() != 'windows'
    package_rpaths += ['-Wl,-rpath,${libdir}']
    foreach runtime_dir : runtime_dirs + serif_runtime_dirs
        argument = '-Wl,-rpath,' + runtime_dir
        if not package_rpaths.contains(argument)
            package_rpaths += [argument]
        endif
    endforeach
endif
if not bundle
pkg.generate(
  libRayFem,
  name: 'rayfem',
  description: 'MFEM field streaming, visualization, and analysis',
  filebase: 'librayfem',
  version: meson.project_version(),
  subdirs: '.',
  # Keep the original external MFEM package public: Meson's pkgconfig module
  # makes nested external dependencies of a declare_dependency private.
  libraries: [mfem_pkg_dep, raylib_dep, czmq_dep] + package_rpaths,
  extra_cflags: ['-std=c++23'] + mfem_exported_flags,
  variables: {'viewer': get_option('prefix') / get_option('bindir') / 'RayFEM'},
)
endif

summary({'MFEM provider': mfem_provider, 'SERiF archive reader': serif_enabled,
         'viewer': build_viewer, 'examples': build_examples, 'tests': build_tests,
         'Python package': get_option('python').enabled(), 'wheel bundle': bundle},
        section: 'RayFEM', bool_yn: true)
