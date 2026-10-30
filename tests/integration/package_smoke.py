#!/usr/bin/env python3
"""Compile and run a consumer using only an installed librayfem.pc package."""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

SOURCE = r'''
#include <libRayfem/init.hpp>
#include <libRayfem/network/client.hpp>
#include <libRayfem/mesh/converter.hpp>
#include <libRayfem/mesh/field_range.hpp>
#include <libRayfem/io/serif.hpp>
#include <cmath>
#include <iostream>
#if defined(MFEM_USE_OPENMP) && !defined(_OPENMP)
#error Installed RayFEM metadata dropped required MFEM OpenMP compiler flags
#endif
int main() {
    if (mfem::GetVersion() != MFEM_VERSION) {
        std::cerr << "MFEM headers do not match the linked library\n";
        return 2;
    }
    auto mesh = mfem::Mesh::MakeCartesian2D(1,1,mfem::Element::QUADRILATERAL,true,1,1);
    mesh.SetCurvature(3);
    mfem::H1_FECollection collection(3,2);
    mfem::FiniteElementSpace space(&mesh,&collection);
    mfem::GridFunction field(&space);
    mfem::FunctionCoefficient coefficient([](const mfem::Vector& x){ return x[0]+2*x[1]; });
    field.ProjectCoefficient(coefficient);
    const auto surface=RayFEM::BuildSurface(mesh,&field);
    const auto range=RayFEM::EvaluateFieldRange(mesh,field);
    if (!std::isfinite(range.min_value) || !std::isfinite(range.max_value) ||
        range.min_value >= range.max_value || range.element_count != 1) return 3;
    RayFEM::WindowParams invalid;invalid.port=0;
    if(surface.values.empty() || RayFEM::init(invalid).status!=RayFEM::InitStatus::Failed) return 1;
    std::cout << "Installed RayFEM public API works with MFEM " << MFEM_VERSION_STRING << '\n';
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prefix", type=Path)
    parser.add_argument("--compiler", default="g++")
    args = parser.parse_args()
    prefix = args.prefix.resolve()
    environment = os.environ.copy()
    environment["PKG_CONFIG_PATH"] = os.pathsep.join([
        str(prefix / "lib/pkgconfig"), str(prefix / "lib64/pkgconfig"),
        environment.get("PKG_CONFIG_PATH", ""),
    ])
    for name in ("LD_LIBRARY_PATH", "DYLD_LIBRARY_PATH", "DYLD_FALLBACK_LIBRARY_PATH"):
        environment.pop(name, None)
    package = subprocess.check_output(
        ["pkg-config", "librayfem", "--cflags", "--libs"], env=environment, text=True)
    flags = shlex.split(package)
    # A system SDK or deliberately external MFEM prefix is allowed; the
    # generated build tree and uninstalled sibling subprojects are not.
    for flag in flags:
        if "__CMake_build" in flag or "/meson-private/" in flag:
            raise RuntimeError("Installed package leaked a build directory: " + flag)
    with tempfile.TemporaryDirectory(prefix="rayfem installed consumer ") as directory:
        folder = Path(directory)
        source = folder / "test.cpp"
        source.write_text(SOURCE)
        for label, arguments in (
            ("requested libs-before-source", [*flags, "--std=c++23", str(source)]),
            ("portable source-before-libs", ["--std=c++23", str(source), *flags]),
        ):
            executable = folder / label.replace(" ", "-")
            subprocess.run([args.compiler, *arguments, "-o", str(executable)], env=environment, check=True)
            subprocess.run([str(executable)], env=environment, check=True)
            print("PASS", label)
    viewer = prefix / "bin/RayFEM"
    if viewer.exists():
        subprocess.run([str(viewer), "--help"], env=environment, check=True, stdout=subprocess.DEVNULL)
        print("PASS installed viewer starts without library-path overrides")


if __name__ == "__main__":
    main()
