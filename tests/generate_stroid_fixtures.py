#!/usr/bin/env python3
"""Generate real Stroid fixtures without changing the external Stroid checkout."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess


COMMON = """order = 5
core_mapping = "multi_block"
r_core = 0.35
r_star = 1.0
r_infinity = 1.8
flattening = 0.25
core_id = 11
envelope_id = 17
vacuum_id = 23
surface_bdr_id = 31
inf_bdr_id = 37

[main.optimization_methods]
tmop = false
smoothstep = true
"""
CONFIGURATIONS = {
    "stroid-order5": "[main]\nrefinement_levels = 1\ninclude_external_domain = false\n" + COMMON,
    "stroid-order5-nc": (
        "[main]\nrefinement_levels = 2\nvacuum_refinement_levels = 0\n"
        "vacuum_outer_refinement_levels = 1\ninclude_external_domain = true\n" + COMMON
    ),
}


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stroid", type=Path, help="Existing Stroid CLI executable")
    parser.add_argument("--output", type=Path, default=Path("build-rayfem/test-output"))
    args = parser.parse_args()
    binary = args.stroid.expanduser().resolve(strict=True)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    version = subprocess.run([str(binary), "info", "--version"], check=True,
                             capture_output=True, text=True).stdout.strip()
    source_commit = subprocess.run(["git", "-C", str(binary.parent), "rev-parse", "HEAD"],
                                   capture_output=True, text=True)
    provenance = {
        "generator": str(binary),
        "generator_version": version,
        "generator_sha256": sha256(binary),
        # This identifies the current checkout; the executable hash identifies
        # the actual generator even when it predates that checkout's HEAD.
        "source_checkout_commit": source_commit.stdout.strip() if source_commit.returncode == 0 else None,
        "fixtures": [],
    }
    for name, configuration in CONFIGURATIONS.items():
        config_path = output / (name + ".toml")
        mesh_path = output / (name + ".mesh")
        config_path.write_text(configuration)
        command = [str(binary), "generate", "-c", str(config_path), "-o", str(mesh_path), "mfem"]
        subprocess.run(command, cwd=output, check=True)
        provenance["fixtures"].append({
            "mesh": str(mesh_path), "bytes": mesh_path.stat().st_size,
            "mesh_sha256": sha256(mesh_path), "configuration": configuration, "command": command,
        })
        print(f"Generated {mesh_path.name}: {mesh_path.stat().st_size:,} bytes")
    destination = output / "stroid-provenance.json"
    destination.write_text(json.dumps(provenance, indent=2) + "\n")
    print(f"Saved generator/configuration provenance to {destination}")


if __name__ == "__main__":
    main()
