c = meson.get_compiler('c')
gl_dep = c.find_library('OpenGL32')

dependencies = [
    gl_dep,
    easy_physics_dep,
    fontstash_dep,
    glfw_dep,
    glew_dep,
    imgui_dep,
    cuber_dep,
]

easy_physics_common_inc = include_directories('include')
easy_physics_common_lib = static_library(
    'easy_physics_common',
    [
        'src/common.cpp',
        'src/GL1Renderer.cpp',
        'src/Gl1FontStashRenderer.cpp',
        'src/Gl3FontStashRenderer.cpp',
        'src/PhysicsScene.cpp',
        'src/ScreenCamera.cpp',
        'src/PhysicsSceneSelector.cpp',
        'src/GlfwPlatform.cpp',
    ],
    include_directories: easy_physics_common_inc,
    dependencies: dependencies,
)
easy_physics_common_dep = declare_dependency(
    include_directories: easy_physics_common_inc,
    link_with: easy_physics_common_lib,
    dependencies: dependencies,
)
