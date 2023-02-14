c = meson.get_compiler('c')
gl_dep = c.find_library('OpenGL32')

easy_physics_common_inc = include_directories('include')
easy_physics_common_lib = static_library(
    'easy_physics_common',
    [
        'src/common.cpp',
        'src/ctrl_func.cpp',
        'src/font_render_func.cpp',
        'src/render_func.cpp',
    ],
    include_directories: easy_physics_common_inc,
    dependencies: [gl_dep, easy_physics_dep],
)
easy_physics_common_dep = declare_dependency(
    include_directories: easy_physics_common_inc,
    link_with: easy_physics_common_lib,
    dependencies: [gl_dep, easy_physics_dep],
)