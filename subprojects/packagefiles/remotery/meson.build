project('remotery', 'c')
remotery_inc = include_directories('lib')
remotery_lib = static_library('remotery', [
    'lib/Remotery.c'
])
remotery_dep = declare_dependency(
    include_directories: remotery_inc,
    link_with: remotery_lib,
)
