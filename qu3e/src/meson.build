qu3e_lib = static_library(
    'qu3e',
    [
        'math/q3Mat3.cpp',
        'math/q3Quaternion.cpp',
        'scene/q3Scene.cpp',
        'scene/q3Body.cpp',
        'scene/q3Box.cpp',
        'dynamics/q3BroadPhase.cpp',
        'dynamics/q3ContactManager.cpp',
        'dynamics/q3ContactSolver.cpp',
        'dynamics/q3ContactConstraint.cpp',
        'dynamics/q3Island.cpp',
        'dynamics/q3Manifold.cpp',
        'dynamics/q3TimeStep.cpp',
        'q3Render.cpp',
    ],
    dependencies: [remotery_dep],
)
qu3e_dep = declare_dependency(
    include_directories: include_directories('.'),
    link_with: qu3e_lib,
)
