glfw_dep = dependency('glfw3', default_options: ['install=true'])
gl_dep = dependency('opengl')
executable('demo', [
    'main.cpp',
    'Demo.cpp',
    'Clock.cpp',
    'App.cpp',
],
    install: true,
    dependencies: [qu3e_dep, glfw_dep, imgui_dep, gl_dep]
)
