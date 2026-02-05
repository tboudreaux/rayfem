config_p = subproject('libconfig',
default_options:[
  'default_library=static',
  'pkg_config=false',
  'build_tests=false',
  'build_examples=false'
])
dependencies += config_p.get_variable('config_dep')
