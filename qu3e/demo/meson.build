glfw_dep = dependency('glfw3', default_options: ['install=true'])
gl_dep = dependency('opengl')
imgui_dep = dependency('imgui', default_options: ['default_library=static'])
executable('demo', [
    'main.cpp',
    'App.cpp',
    'demos/Demo.cpp',
    'demos/Clock.cpp',
],
    install: true,
    cpp_args: ['-D_CRT_SECURE_NO_WARNINGS'],
    dependencies: [qu3e_dep, glfw_dep, imgui_dep, gl_dep]
)
