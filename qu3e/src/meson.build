qu3e_lib = static_library('qu3e', [
    'broadphase/q3BroadPhase.cpp',
    'broadphase/q3DynamicAABBTree.cpp',
    'collision/q3Box.cpp',
    'common/q3Geometry.cpp',
    'common/q3Memory.cpp',
    'dynamics/q3Body.cpp',
    'dynamics/q3Contact.cpp',
    'dynamics/q3ContactManager.cpp',
    'dynamics/q3ContactSolver.cpp',
    'dynamics/q3Island.cpp',
    'math/q3Mat3.cpp',
    'math/q3Quaternion.cpp',
    'scene/q3Scene.cpp',
    'debug/q3Render.cpp',
],
    dependencies: [remotery_dep],
)
qu3e_dep = declare_dependency(
    include_directories: include_directories('.'),
    link_with: qu3e_lib,
)
