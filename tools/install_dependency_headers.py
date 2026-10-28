#!/usr/bin/env python3
"""Install public headers omitted by Meson's CMake target importer."""
import argparse
import os
from pathlib import Path
import shutil


def source_directory(build):
    for line in (build / "CMakeCache.txt").read_text().splitlines():
        if line.startswith("CMAKE_HOME_DIRECTORY:INTERNAL="):
            return Path(line.split("=", 1)[1])
    raise RuntimeError("CMake source directory is missing from " + str(build))


def copy(source, target):
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--includedir", required=True, type=Path)
    parser.add_argument("--raylib-build", required=True, type=Path)
    parser.add_argument("--czmq-build", required=True, type=Path)
    parser.add_argument("--mfem-build", type=Path)
    args = parser.parse_args()
    if args.includedir.is_absolute():
        include = Path(os.environ.get("DESTDIR", "/")) / str(args.includedir).lstrip("/")
    else:
        include = Path(os.environ["MESON_INSTALL_DESTDIR_PREFIX"]) / args.includedir
    raylib = source_directory(args.raylib_build)
    for name in ("raylib.h", "raymath.h", "rlgl.h"):
        copy(raylib / "src" / name, include / name)
    czmq = source_directory(args.czmq_build)
    for header in (czmq / "include").glob("*.h"):
        copy(header, include / header.name)
    if args.mfem_build:
        mfem = source_directory(args.mfem_build)
        for name in ("mfem.hpp", "mfem-performance.hpp"):
            copy(args.mfem_build / "InstallHeaders" / name, include / name)
            copy(mfem / name, include / "mfem" / name)
        for directory in ("general", "linalg", "mesh", "fem", "config"):
            for header in (mfem / directory).rglob("*.hpp"):
                copy(header, include / "mfem" / header.relative_to(mfem))
        copy(args.mfem_build / "config/_config.hpp", include / "mfem/config/_config.hpp")
    print("Installed public dependency headers in", include)


if __name__ == "__main__":
    main()
