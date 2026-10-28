#!/usr/bin/env python3
"""Verify a Meson parent shares its MFEM/SERiF libraries with a RayFEM subproject.

Uses already built dependencies only. All generated projects and build outputs
live in a temporary directory; no external dependency checkout is modified.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile

CONSUMER = r'''
#include <libRayfem/mesh/converter.hpp>
#include <libRayfem/io/serif.hpp>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <set>
#include <stdexcept>
#if defined(MFEM_USE_OPENMP) && !defined(_OPENMP)
#error Parent dependency dropped required MFEM OpenMP compiler flags
#endif
#if defined(__APPLE__)
#include <mach-o/dyld.h>
#elif defined(__linux__)
#include <link.h>
#endif

std::set<std::filesystem::path> loaded_mfem() {
    std::set<std::filesystem::path> result;
    const auto record=[&](const char* name) {
        if(!name || !*name)return;
        const std::filesystem::path path(name);
        if(path.filename().string().starts_with("libmfem"))
            result.insert(std::filesystem::weakly_canonical(path));
    };
#if defined(__APPLE__)
    for(std::uint32_t i=0;i<_dyld_image_count();++i)record(_dyld_get_image_name(i));
#elif defined(__linux__)
    struct Visitor { decltype(record)* callback; } visitor{&record};
    dl_iterate_phdr([](struct dl_phdr_info* info,std::size_t,void* argument){
        (*static_cast<Visitor*>(argument)->callback)(info->dlpi_name);return 0;
    },&visitor);
#else
    throw std::runtime_error("Runtime MFEM library enumeration requires macOS or Linux");
#endif
    return result;
}
int main(int argc,char** argv) {
    if(argc!=3)return 2;
    if(mfem::GetVersion()!=MFEM_VERSION)
        throw std::runtime_error("Parent MFEM headers do not match its linked library");
    const bool expect_serif=std::string(argv[2])=="true";
    auto mesh=mfem::Mesh::MakeCartesian2D(2,2,mfem::Element::QUADRILATERAL,true,1,1);
    mesh.SetCurvature(3);
    mfem::H1_FECollection collection(3,2);
    mfem::FiniteElementSpace space(&mesh,&collection);
    mfem::GridFunction field(&space);
    mfem::FunctionCoefficient linear([](const mfem::Vector& p){return p[0]+2*p[1];});
    field.ProjectCoefficient(linear);
    const auto surface=RayFEM::BuildSurface(mesh,&field);
    if(surface.values.empty() || std::abs(surface.min_value)>1e-12 || std::abs(surface.max_value-3)>1e-12)
        throw std::runtime_error("Parent-created MFEM field failed RayFEM's cross-library evaluation");
    if(RayFEM::SerifLoadingAvailable()!=expect_serif)
        throw std::runtime_error("Parent-provided SERiF override was not honored");
    const auto libraries=loaded_mfem();
    if(libraries.size()!=1)throw std::runtime_error("Expected exactly one dynamically loaded MFEM library");
    const auto expected=std::filesystem::weakly_canonical(std::filesystem::path(argv[1])/"lib");
    if(libraries.begin()->parent_path()!=expected)
        throw std::runtime_error("RayFEM loaded MFEM outside the parent's prefix");
    std::cout<<"MFEM_LOADED="<<libraries.begin()->string()<<'\n';
    std::cout<<"PASS parent MFEM field evaluated across RayFEM; one shared MFEM; SERiF="<<expect_serif<<'\n';
}
'''


def meson_string(value):
    return "'" + str(value).replace("\\", "\\\\").replace("'", "\\'") + "'"


def project(prefix, serif_source, serif_build):
    serif = ""
    if serif_source:
        includes = [serif_source / "src/include", serif_build / "src/include",
                    serif_source / "build-config/nameof/include", serif_source / "build-config/magicenum/include"]
        serif = """
if get_option('with_serif')
    serif_library = cpp.find_library('serif', dirs: [@LIBDIR@], static: false)
    serif_dependency = declare_dependency(
        dependencies: [serif_library, parent_mfem],
        compile_args: ['-DMAGIC_ENUM_RANGE_MAX=2000', @INCLUDES@],
        variables: {'mfem_prefix': mfem_prefix, 'libdir': @LIBDIR@})
    meson.override_dependency('serif', serif_dependency)
endif
""".replace("@LIBDIR@", meson_string(serif_build / "src")).replace(
            "@INCLUDES@", ", ".join(meson_string("-I" + str(path)) for path in includes))
    return """
project('rayfem_parent_check', ['cpp', 'c'], default_options: ['cpp_std=c++23'])
cpp = meson.get_compiler('cpp')
mfem_prefix = @PREFIX@
mfem_installation = dependency('MFEM', method: 'cmake', modules: ['mfem'],
    cmake_args: ['-DMFEM_DIR=' + mfem_prefix / 'lib/cmake/mfem'], allow_fallback: false)
parent_mfem = declare_dependency(dependencies: mfem_installation,
    variables: {'prefix': mfem_prefix,
                'MFEM_CXX_FLAGS': mfem_installation.get_variable(cmake: 'MFEM_CXX_FLAGS', default_value: '')})
meson.override_dependency('mfem', parent_mfem)
@SERIF@
# Deliberately keep viewer/examples/tests at auto: these must all default off
# when RayFEM is consumed as a subproject.
rayfem = subproject('rayfem')
assert(not rayfem.get_variable('build_viewer'), 'Subproject enabled its viewer')
assert(not rayfem.get_variable('build_examples'), 'Subproject enabled its examples')
assert(not rayfem.get_variable('build_tests'), 'Subproject enabled its tests')
assert(rayfem.get_variable('mfem_provider') == 'parent or installed dependency',
       'RayFEM did not reuse the parent MFEM dependency')
assert(rayfem.get_variable('serif_enabled') == get_option('with_serif'),
       'RayFEM did not honor the parent SERiF dependency')
consumer = executable('parent_consumer', 'consumer.cpp',
    dependencies: [parent_mfem, rayfem.get_variable('rayfem_dep')])
""".replace("@PREFIX@", meson_string(prefix)).replace("@SERIF@", serif)


def run(arguments, environment, log):
    with log.open("w") as output:
        result = subprocess.run(arguments, env=environment, stdout=output, stderr=subprocess.STDOUT)
    if result.returncode:
        print(log.read_text()[-18000:])
        raise RuntimeError(f"Command failed ({result.returncode}); log: {log}")


def check_build(build, environment):
    targets = json.loads(subprocess.check_output(["meson", "introspect", str(build), "--targets"], env=environment))
    forbidden = {"RayFEM", "exampleClient", "sandbox", "mfem"}
    for target in targets:
        if target["name"] in forbidden or target["name"].endswith("_test"):
            raise RuntimeError("Unexpected subproject app/test/bundled MFEM target: " + target["name"])
    commands = json.loads((build / "compile_commands.json").read_text())
    mfem_sources = [entry["file"] for entry in commands if "/subprojects/mfem/" in entry["file"]]
    if mfem_sources:
        raise RuntimeError("Bundled MFEM source compilation was introduced: " + mfem_sources[0])
    print("PASS auto subproject settings: no viewer, examples, tests, or bundled MFEM compilation", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mfem-prefix", type=Path, required=True)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--serif-source", type=Path)
    parser.add_argument("--serif-build", type=Path)
    parser.add_argument("--cc")
    parser.add_argument("--cxx")
    parser.add_argument("--jobs", type=int, default=6)
    parser.add_argument("--keep", action="store_true", help="Keep the temporary build and logs after finishing")
    args = parser.parse_args()
    repo, prefix = args.repo.resolve(), args.mfem_prefix.resolve()
    serif_source = args.serif_source.resolve() if args.serif_source else None
    serif_build = (args.serif_build or serif_source / "build").resolve() if serif_source else None
    if not (prefix / "lib/cmake/mfem/MFEMConfig.cmake").is_file():
        parser.error("--mfem-prefix must contain an existing MFEM CMake installation")
    environment = os.environ.copy()
    for name in ("CPATH", "C_INCLUDE_PATH", "CPLUS_INCLUDE_PATH", "LD_LIBRARY_PATH",
                 "DYLD_LIBRARY_PATH", "DYLD_FALLBACK_LIBRARY_PATH"):
        environment.pop(name, None)
    for name, value in (("CC", args.cc), ("CXX", args.cxx)):
        if value:
            environment[name] = value
    environment["PKG_CONFIG_PATH"] = os.pathsep.join(filter(None, [str(repo / ".rayfem-deps/lib/pkgconfig"), environment.get("PKG_CONFIG_PATH")]))
    environment["CMAKE_PREFIX_PATH"] = os.pathsep.join(filter(None, [str(prefix), str(repo / ".rayfem-deps"), environment.get("CMAKE_PREFIX_PATH")]))
    environment.setdefault("CCACHE_DIR", str(Path(tempfile.gettempdir()) / "rayfem-parent-ccache"))
    temporary = tempfile.TemporaryDirectory(prefix="rayfem parent ") if not args.keep else None
    root = Path(temporary.name if temporary else tempfile.mkdtemp(prefix="rayfem parent "))
    print("Parent fixture:", root, flush=True)
    try:
        (root / "subprojects").mkdir()
        (root / "subprojects/rayfem").symlink_to(repo, target_is_directory=True)
        (root / "consumer.cpp").write_text(CONSUMER)
        (root / "meson.build").write_text(project(prefix, serif_source, serif_build))
        (root / "meson.options").write_text("option('with_serif', type: 'boolean', value: false)\n")
        build = root / "build"
        for with_serif in ([False, True] if serif_source else [False]):
            label = "with-serif" if with_serif else "mfem-only"
            command = ["meson", "setup", str(build), str(root), "-Dwith_serif=" + str(with_serif).lower(), "-Dbuildtype=debugoptimized"]
            if with_serif:
                command.append("--reconfigure")
            run(command, environment, root / (label + "-configure.log"))
            check_build(build, environment)
            run(["ninja", "-C", str(build), "parent_consumer", "-j", str(args.jobs)], environment, root / (label + "-build.log"))
            # Inspect the process's actual loaded images, not just linker flags.
            result = subprocess.run([str(build / "parent_consumer"), str(prefix), str(with_serif).lower()], env=environment, text=True, capture_output=True)
            print(result.stdout, end="", flush=True)
            if result.returncode:
                print(result.stderr)
                raise RuntimeError("Parent consumer runtime failed")
        print("Parent integration checks passed", flush=True)
    finally:
        if temporary:
            temporary.cleanup()
        else:
            print("Retained fixture:", root, flush=True)


if __name__ == "__main__":
    main()
