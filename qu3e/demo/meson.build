directxmath_dep = dependency('directxmath')
imgui_dep = dependency('imgui', default_options: ['default_library=static'])
cuber_gl3_dep = dependency('cuber_gl3')
executable('demo', [
    'main.cpp',
    'App.cpp',
    'GL2Renderer.cpp',
    'GL3Renderer.cpp',
    'Demos/Test.cpp',
    'Demos/RayPush.cpp',
    'Demos/DropBoxes.cpp',
    'Demos/BoxStack.cpp',
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
        cuber_gl3_dep,
    ]
)
