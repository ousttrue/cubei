c = meson.get_compiler('c')
gl_dep = c.find_library('OpenGL32')

dependencies = [
    gl_dep,
    easy_physics_dep,
    fontstash_dep,
]

easy_physics_common_inc = include_directories('include')
easy_physics_common_lib = static_library(
    'easy_physics_common',
    [
        'src/common.cpp',
        'src/ctrl_func.cpp',
        'src/font_render_func.cpp',
        'src/GL1Renderer.cpp',
        'src/physics_scene.cpp',
        'src/FontStashRenderer.cpp',
        'src/GraphicsScene.cpp',
    ],
    include_directories: easy_physics_common_inc,
    dependencies: dependencies,
)
easy_physics_common_dep = declare_dependency(
    include_directories: easy_physics_common_inc,
    link_with: easy_physics_common_lib,
    dependencies: dependencies,
)
