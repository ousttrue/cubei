project('imgui', 'cpp')
glfw_dep = dependency('glfw3', default_options: ['install=true'])
imgui_lib = static_library('imgui', [
    'imgui.cpp',
    'imgui_draw.cpp',
    'imgui_widgets.cpp',
    'imgui_tables.cpp',
    'imgui_demo.cpp',
    'backends/imgui_impl_opengl3.cpp',
    'backends/imgui_impl_glfw.cpp',
],
    dependencies: [glfw_dep])
imgui_dep = declare_dependency(
    include_directories: include_directories('.', 'backends'),
    link_with: imgui_lib,
)
