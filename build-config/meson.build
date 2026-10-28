cmake = import('cmake')

# Resolve the external reader first so it can select the exact MFEM ABI used
# by its existing shared library. Neither external project is built here.
subdir('serif')
subdir('mfem')
subdir('raylib')
subdir('czmq')
if get_option('glvis_experiment')
    subdir('glvis')
endif
if build_viewer or build_examples or get_option('glvis_experiment')
    subdir('CLI11')
endif
if build_ui
    subdir('imgui')
    subdir('rlImGui')
endif

subdir('system')
