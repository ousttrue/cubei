plog_dep = dependency('plog')
directxmath_dep = dependency('directxmath')
glfw_dep = dependency('glfw3', default_options: ['install=true'])
gl_dep = dependency('opengl')
glew_dep = dependency('glew')
imgui_dep = dependency('imgui', default_options: ['default_library=static'])
executable('demo', [
    'main.cpp',
    'App.cpp',
    'GL2Renderer.cpp',
    'GL3Renderer.cpp',
],
    install: true,
    cpp_args: ['-D_CRT_SECURE_NO_WARNINGS'],
    dependencies: [
        qu3e_dep, 
        glfw_dep, 
        imgui_dep, 
        gl_dep, 
        plog_dep, 
        glew_dep, 
        directxmath_dep,
        remotery_dep,
    ]
)
