project('RayFEM', ['cpp', 'c'],
        version : '0.1.0',
        default_options : ['warning_level=3', 'cpp_std=c++23'])

c_args = []
cpp_args = []
link_args = []
name_suffix = []
dependencies = []

add_project_arguments('-U_FORTIFY_SOURCE', '-D_FORTIFY_SOURCE=0', language: 'cpp')

cc = meson.get_compiler('c')
cxx = meson.get_compiler('cpp')

subdir('build-config')
subdir('src')