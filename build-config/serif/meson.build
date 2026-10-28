serif_enabled = false
serif_dep = declare_dependency()
serif_runtime_dirs = []
serif_mfem_prefix = ''
serif_source_dir = get_option('serif_source_dir')
serif_build_dir = get_option('serif_build_dir')

if not get_option('serif').disabled()
    existing_serif = dependency('serif', required: false, allow_fallback: false)
    if existing_serif.found()
        serif_dep = existing_serif
        serif_enabled = true
        serif_mfem_prefix = serif_dep.get_variable(internal: 'mfem_prefix', pkgconfig: 'mfem_prefix', default_value: '')
        serif_library_dir = serif_dep.get_variable(internal: 'libdir', pkgconfig: 'libdir', default_value: '')
        if serif_library_dir != ''
            serif_runtime_dirs += [serif_library_dir]
        endif
    endif
endif

if not serif_enabled and not get_option('serif').disabled() and (get_option('serif').enabled() or serif_source_dir != '')
    if serif_source_dir == ''
        error('SERiF support needs serif_source_dir naming an already-built external checkout.')
    endif
    if not fs.is_absolute(serif_source_dir)
        serif_source_dir = meson.project_source_root() / serif_source_dir
    endif
    if serif_build_dir == ''
        serif_build_dir = serif_source_dir / 'build'
    endif
    if not fs.is_absolute(serif_build_dir)
        serif_build_dir = meson.project_source_root() / serif_build_dir
    endif
    serif_mfem_prefix = get_option('mfem_prefix')
    if serif_mfem_prefix == ''
        serif_mfem_prefix = serif_build_dir / 'build-config/mfem/mfem-prefix'
    elif not fs.is_absolute(serif_mfem_prefix)
        serif_mfem_prefix = meson.project_source_root() / serif_mfem_prefix
    endif
    if not fs.is_file(serif_source_dir / 'src/include/serif/utils/serialize/archive.hpp') or not fs.is_file(serif_mfem_prefix / 'lib/cmake/mfem/MFEMConfig.cmake')
        error('SERiF headers and its existing MFEM installation must already exist. Set mfem_prefix for a system-provider SERiF build; RayFEM never builds SERiF.')
    endif
    serif_library = cxx.find_library('serif', dirs: [serif_build_dir / 'src'], required: true, static: false)
    serif_dep = declare_dependency(dependencies: serif_library,
        compile_args: ['-DMAGIC_ENUM_RANGE_MAX=2000',
                      '-I' + serif_source_dir / 'src/include',
                      '-I' + serif_build_dir / 'src/include',
                      '-I' + serif_source_dir / 'build-config/nameof/include',
                      '-I' + serif_source_dir / 'build-config/magicenum/include'])
    serif_runtime_dirs = [serif_build_dir / 'src', serif_mfem_prefix / 'lib']
    serif_enabled = true
endif
