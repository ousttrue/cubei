easy_physics_lib = static_library(
    'easy_physics',
    [
        'elements/EpxContact.cpp',
        'elements/EpxConvexMesh.cpp',
        'collision/EpxClosestFunction.cpp',
        'collision/EpxConvexConvexContact.cpp',
        'pipeline/EpxBroadphase.cpp',
        'pipeline/EpxConstraintSolver.cpp',
        'pipeline/EpxCollisionDetection.cpp',
        'pipeline/EpxIntegrate.cpp',
    ],
)
easy_physics_dep = declare_dependency(
    link_with: easy_physics_lib,
    include_directories: include_directories('.'),
)
