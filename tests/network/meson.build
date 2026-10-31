# Streaming, the viewer launcher and shutdown over the network.

streaming_test = executable('streaming_test', 'streaming_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('streaming', streaming_test,
    args: [meson.project_source_root() / 'subprojects/mfem/data/square-nurbs.mesh'], timeout: 120)

launcher_test = executable('launcher_test', 'launcher_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('launcher', launcher_test, timeout: 30)

shutdown_test = executable('shutdown_test', 'shutdown_test.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
test('shutdown', shutdown_test, timeout: 30)

# Not a test: how long a solver waits on each send. Run it by hand.
executable('stream_benchmark', 'stream_benchmark.cpp',
    dependencies: [rayfem_dep, app_dependencies], link_args: link_args)
