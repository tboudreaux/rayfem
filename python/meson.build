# The rayfem Python package: a nanobind extension on the stable ABI (one
# wheel for CPython 3.12 and later) over the MFEM-free client core, plus pure
# Python modules. nanobind comes from the build environment (pip install
# nanobind, or pyproject's build requirements) and is compiled here.
nanobind_root = run_command(python_build, '-c',
    'import nanobind, os; print(os.path.dirname(nanobind.__file__))', check: true).stdout().strip()
# Stable ABI 3.12 and RayFEM's own type domain, so no type registered here
# can collide with another nanobind extension (SERiF's, for example).
nanobind_args = ['-DPy_LIMITED_API=0x030C0000', '-DNB_DOMAIN=rayfem']
nanobind_includes = include_directories(nanobind_root / 'include', nanobind_root / 'ext/robin_map/include', is_system: true)
python_embed = python_build.dependency(embed: false)
nanobind_lib = static_library('nanobind-abi3', nanobind_root / 'src/nb_combined.cpp',
    include_directories: nanobind_includes,
    dependencies: python_embed,
    cpp_args: nanobind_args + ['-DNB_BUILD', '-fno-strict-aliasing', '-w'],
    gnu_symbol_visibility: 'hidden',
    pic: true)

python_build.extension_module('_rayfem', 'module.cpp',
    include_directories: nanobind_includes,
    dependencies: [rayfem_client_dep],
    link_with: nanobind_lib,
    cpp_args: cpp_args + ['-DNB_DOMAIN=rayfem'],
    gnu_symbol_visibility: 'hidden',
    limited_api: '3.12',
    install: true,
    subdir: 'rayfem')

python_build.install_sources(
    'rayfem/__init__.py', 'rayfem/_viewer.py', 'rayfem/mesh.py', 'rayfem/py.typed',
    'rayfem/adapters/__init__.py', 'rayfem/adapters/pymfem.py',
    preserve_path: true)
