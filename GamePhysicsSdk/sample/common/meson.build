c = meson.get_compiler('c')
gl_dep = c.find_library('OpenGL32')

easy_physics_common_lib = static_library(
    'easy_physics_common',
    [
        'common.cpp',
        'ctrl_func.cpp',
        'font_render_func.cpp',
        'render_func.cpp',
    ],
)
easy_physics_common_dep = declare_dependency(
    include_directories: include_directories('.'),
    link_with: easy_physics_common_lib,
    dependencies: [gl_dep],
)
