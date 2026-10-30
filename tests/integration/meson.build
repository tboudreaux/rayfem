# Programs and scripts that check RayFEM against a running viewer, the
# examples, an installed package or a parent project.

# A 2D heat equation streamed to a viewer that the rayfem Python package
# starts; run python/tests/test_diffusion_stream.py with this program.
diffusion_stream = executable('diffusion_stream', 'diffusion_stream.cpp',
    dependencies: [rayfem_dep], link_args: link_args)

# Prints what the server decodes; Python tests use it to check sends end to end.
stream_probe = executable('stream_probe', 'stream_probe.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)

# Needs fixtures from generate_stroid_fixtures.py (see README).
executable('stroid_flow_test', 'stroid_flow_test.cpp', dependencies: [rayfem_dep, app_dependencies], link_args: link_args)

# Starts, reuses and shuts down a real viewer window; pass its path.
executable('viewer_lifecycle_test', 'viewer_lifecycle_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)

# The real example checks heat conservation and moving-domain area each step,
# plus L2-energy decay when the mesh is fixed. No network or window required.
if build_examples
    test('sandbox_pulsating', sandbox,
    args: ['--no-stream', '--resolution', '5', '--steps', '40',
           '--dt', '0.05', '--period', '1', '--amplitude', '0.25'], timeout: 60)
test('sandbox_fixed', sandbox,
    args: ['--no-stream', '--resolution', '5', '--steps', '40',
           '--dt', '0.05', '--amplitude', '0'], timeout: 60)

endif
